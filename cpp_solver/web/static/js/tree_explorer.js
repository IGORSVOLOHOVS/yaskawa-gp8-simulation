// Yaskawa GP8 Physical Tree Explorer & Technical Passport Engine
let allComponents = [];
const compMap = {};
const childrenMap = {};

async function initTreeExplorer() {
    try {
        let spatialManifest = {};
        try {
            const manifestRes = await fetch('/api/robot/spatial_manifest');
            if (manifestRes.ok) {
                spatialManifest = await manifestRes.json();
                window.spatialManifest = spatialManifest;
            }
        } catch (e) {
            console.warn('Could not fetch spatial manifest, using fallbacks:', e);
        }

        const res = await fetch('/api/robot/tree');
        const data = await res.json();
        allComponents = data.components || [];

        // Build Index Maps
        allComponents.forEach(comp => {
            compMap[comp.id] = comp;
            if (!childrenMap[comp.parent_id]) childrenMap[comp.parent_id] = [];
            childrenMap[comp.parent_id].push(comp);
        });

        // Register 3D Procedural Meshes
        ensureAll1093ComponentsHave3DMeshes(spatialManifest);

        // Render Tree
        renderTree(childrenMap['root'] || []);

        const countBadge = document.getElementById('total-count-badge');
        if (countBadge) countBadge.textContent = `${allComponents.length} Parts`;

        // Select Root by default
        if (allComponents.length > 0) {
            selectPart('sec_1_casting');
        }
    } catch (err) {
        console.error('Failed to load physical robot tree:', err);
    }
}

function renderTree(nodes, container = document.getElementById('tree-root')) {
    container.innerHTML = '';
    nodes.forEach(node => {
        container.appendChild(createTreeNodeElement(node));
    });
}

function createTreeNodeElement(comp) {
    const hasChildren = (childrenMap[comp.id] && childrenMap[comp.id].length > 0);
    const nodeDiv = document.createElement('div');
    nodeDiv.className = 'tree-node';
    nodeDiv.id = `tree-node-${comp.id}`;

    const labelDiv = document.createElement('div');
    labelDiv.className = 'node-label';
    labelDiv.id = `node-label-${comp.id}`;

    // Separate click on arrow toggle vs click on label
    const toggleSpan = document.createElement('span');
    toggleSpan.className = 'node-toggle';
    toggleSpan.textContent = hasChildren ? '▶' : ' ';
    toggleSpan.onclick = (e) => {
        e.stopPropagation();
        if (!hasChildren) return;
        const childContainer = nodeDiv.querySelector(`.child-container-${comp.id}`);
        if (childContainer) {
            const isHidden = childContainer.style.display === 'none';
            childContainer.style.display = isHidden ? 'block' : 'none';
            toggleSpan.textContent = isHidden ? '▼' : '▶';
        }
    };

    const iconSpan = document.createElement('span');
    iconSpan.className = 'node-icon';
    iconSpan.textContent = getIconForLevel(comp.level, comp.system_category_id);

    const titleSpan = document.createElement('span');
    titleSpan.className = 'node-title';
    titleSpan.textContent = comp.name;

    labelDiv.appendChild(toggleSpan);
    labelDiv.appendChild(iconSpan);
    labelDiv.appendChild(titleSpan);

    if (hasChildren) {
        const badge = document.createElement('span');
        badge.className = 'count-badge';
        badge.textContent = childrenMap[comp.id].length;
        labelDiv.appendChild(badge);
    }

    const levelBadge = document.createElement('span');
    levelBadge.className = 'level-badge';
    levelBadge.textContent = `L${comp.level}`;
    labelDiv.appendChild(levelBadge);

    labelDiv.onclick = () => selectPart(comp.id);

    nodeDiv.appendChild(labelDiv);

    if (hasChildren) {
        const childContainer = document.createElement('div');
        childContainer.className = `child-container-${comp.id}`;
        childContainer.style.display = 'none';
        childrenMap[comp.id].forEach(child => {
            childContainer.appendChild(createTreeNodeElement(child));
        });
        nodeDiv.appendChild(childContainer);
    }

    return nodeDiv;
}

function getIconForLevel(level, catId) {
    if (level === 1) return '⚙️';
    if (level === 2) return '📦';
    if (level === 3) return '⚡';
    if (level === 4) return '🧩';
    return '🔩';
}

function selectPart(partId) {
    const comp = compMap[partId];
    if (!comp) return;

    selectedPartId = partId;

    // Highlight Active Tree Node
    document.querySelectorAll('.node-label').forEach(el => el.classList.remove('active'));
    const activeLabel = document.getElementById(`node-label-${partId}`);
    if (activeLabel) {
        activeLabel.classList.add('active');
        expandParentsInTree(partId);
    }

    // Render Technical Specs Passport
    renderSpecsPassport(comp);

    // Update Breadcrumb Trail
    updateBreadcrumbs(comp);

    // Turn on dynamic X-Ray if internal component (level >= 3)
    if (comp.level >= 3) {
        toggleXRayMode(true);
    }

    // Trigger Micro-Macro Camera Zoom & Glowing 3D Bounding Box Spotlight
    spotlightSelectedMesh(partId, comp);
}

function expandParentsInTree(partId) {
    let curr = compMap[partId];
    while (curr && curr.parent_id) {
        const parentId = curr.parent_id;
        const parentNode = document.getElementById(`tree-node-${parentId}`);
        if (parentNode) {
            const childContainer = parentNode.querySelector(`.child-container-${parentId}`);
            const toggleSpan = parentNode.querySelector(`.node-toggle`);
            if (childContainer) {
                childContainer.style.display = 'block';
                if (toggleSpan) toggleSpan.textContent = '▼';
            }
        }
        curr = compMap[parentId];
    }
}

function renderSpecsPassport(comp) {
    const specsContainer = document.getElementById('specs-container');
    if (!specsContainer) return;

    specsContainer.innerHTML = `
        <div class="spec-title">${comp.name}</div>
        <div class="spec-part-no">Part #${comp.part_number}</div>

        <div class="specs-grid">
            <div class="grid-item">
                <div class="spec-label">Subsystem Category</div>
                <div class="spec-val" style="color:#38bdf8;">${comp.system_name || comp.category_name || comp.subsystem}</div>
            </div>
            <div class="grid-item">
                <div class="spec-label">Hierarchy Level</div>
                <div class="spec-val-mono">Level ${comp.level}</div>
            </div>
            <div class="grid-item">
                <div class="spec-label">Manufacturer</div>
                <div class="spec-val">${comp.manufacturer}</div>
            </div>
            <div class="grid-item">
                <div class="spec-label">Material / Composition</div>
                <div class="spec-val">${comp.material}</div>
            </div>
        </div>

        <div class="spec-group">
            <div class="spec-label">Technical Specification</div>
            <div class="spec-val">${comp.specs_summary || comp.specifications}</div>
        </div>

        <div class="spec-group">
            <div class="spec-label">Manufacturing Tolerance</div>
            <div class="spec-val-mono">${comp.tolerances || comp.tolerance}</div>
        </div>

        <div class="spec-group" style="margin-top:12px;">
            <div class="spec-label">Database Component Unique ID</div>
            <div class="spec-val-mono" style="font-size:0.75rem; color:#94a3b8;">${comp.id}</div>
        </div>
    `;
}

function updateBreadcrumbs(comp) {
    const trail = [];
    let curr = comp;
    while (curr) {
        trail.unshift(curr.name);
        curr = compMap[curr.parent_id];
    }
    const elem = document.getElementById('breadcrumb-trail');
    if (elem) elem.textContent = trail.join(' > ');
}

function zoomToParentComponent() {
    const currentComp = compMap[selectedPartId];
    if (currentComp && currentComp.parent_id && compMap[currentComp.parent_id]) {
        selectPart(currentComp.parent_id);
    }
}

function filterTree() {
    const query = document.getElementById('search-input').value.toLowerCase().trim();
    const catSelect = document.getElementById('category-filter');
    const selectedCat = catSelect ? parseInt(catSelect.value) : 0;

    if (!query && selectedCat === 0) {
        renderTree(childrenMap['root'] || []);
        return;
    }

    const filtered = allComponents.filter(c => {
        const matchesQuery = !query || (
            c.name.toLowerCase().includes(query) ||
            c.part_number.toLowerCase().includes(query) ||
            c.material.toLowerCase().includes(query) ||
            c.id.toLowerCase().includes(query)
        );
        const matchesCat = (selectedCat === 0) || (c.system_category_id === selectedCat);
        return matchesQuery && matchesCat;
    });

    renderTree(filtered.slice(0, 100));
}

function expandAllTreeNodes() {
    document.querySelectorAll('[class^="child-container-"]').forEach(el => {
        el.style.display = 'block';
    });
    document.querySelectorAll('.node-toggle').forEach(el => {
        if (el.textContent.trim() === '▶') el.textContent = '▼';
    });
}

function collapseAllTreeNodes() {
    document.querySelectorAll('[class^="child-container-"]').forEach(el => {
        el.style.display = 'none';
    });
    document.querySelectorAll('.node-toggle').forEach(el => {
        if (el.textContent.trim() === '▼') el.textContent = '▶';
    });
}
