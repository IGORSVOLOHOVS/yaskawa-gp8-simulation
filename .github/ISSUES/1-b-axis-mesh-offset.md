---
name: Bug report
about: Issue #1 tracking B-axis mesh offset and HTTP response order
title: '[BUG] Joint 5 translation offset and HTTP response header order'
labels: bug
assignees: ''
---

Joint 5 translation offset in server and kinematics is 0.15m. The web handler emits HTTP header bytes before HTTP status code 200.
