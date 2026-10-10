#include "study/learning_models.hpp"

#include "study/gp8_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

namespace yaskawa::study {

namespace {

constexpr double kPi = std::numbers::pi;
constexpr int kSeedMin = 0;
constexpr int kSeedMax = 1000000;

// ---------------------------------------------------------------------------
// neural_network: a two-layer feedforward net trained by backpropagation on
// the inverse kinematics of the GP8's planar L-U sub-chain.
// ---------------------------------------------------------------------------

struct Normaliser {
    double offset = 0.0;
    double scale = 1.0;

    [[nodiscard]] double forward(double raw) const noexcept { return (raw - offset) / scale; }
    [[nodiscard]] double inverse(double normalised) const noexcept {
        return normalised * scale + offset;
    }
};

[[nodiscard]] Normaliser fit_normaliser(const std::vector<double>& values) noexcept {
    Normaliser out;
    if (values.empty()) {
        return out;
    }
    double low = values.front();
    double high = values.front();
    for (const double v : values) {
        low = std::min(low, v);
        high = std::max(high, v);
    }
    out.offset = 0.5 * (low + high);
    out.scale = std::max(0.5 * (high - low), 1e-12);
    return out;
}

// The two-layer network: inputs -> tanh hidden -> linear outputs. Stored as
// plain vectors because the layer width is a parameter of the panel.
struct Network {
    std::size_t inputs = 0;
    std::size_t hidden = 0;
    std::size_t outputs = 0;
    std::vector<double> w1;  // hidden x inputs, row-major
    std::vector<double> b1;  // hidden
    std::vector<double> w2;  // outputs x hidden, row-major
    std::vector<double> b2;  // outputs
};

[[nodiscard]] Network make_network(std::size_t inputs, std::size_t hidden, std::size_t outputs,
                                   sensing::SeededRng& rng) {
    Network net;
    net.inputs = inputs;
    net.hidden = hidden;
    net.outputs = outputs;
    net.w1.resize(hidden * inputs, 0.0);
    net.b1.resize(hidden, 0.0);
    net.w2.resize(outputs * hidden, 0.0);
    net.b2.resize(outputs, 0.0);
    // Xavier/Glorot initialisation: the variance that keeps the signal from
    // vanishing or exploding through a tanh layer.
    const double range1 = std::sqrt(6.0 / static_cast<double>(inputs + hidden));
    const double range2 = std::sqrt(6.0 / static_cast<double>(hidden + outputs));
    for (double& w : net.w1) {
        w = rng.uniform(-range1, range1);
    }
    for (double& w : net.w2) {
        w = rng.uniform(-range2, range2);
    }
    return net;
}

void forward_pass(const Network& net, const std::vector<double>& x, std::vector<double>& hidden,
                  std::vector<double>& output) noexcept {
    for (std::size_t h = 0; h < net.hidden; ++h) {
        double sum = net.b1[h];
        for (std::size_t i = 0; i < net.inputs; ++i) {
            sum += net.w1[h * net.inputs + i] * x[i];
        }
        hidden[h] = std::tanh(sum);
    }
    for (std::size_t o = 0; o < net.outputs; ++o) {
        double sum = net.b2[o];
        for (std::size_t h = 0; h < net.hidden; ++h) {
            sum += net.w2[o * net.hidden + h] * hidden[h];
        }
        output[o] = sum;
    }
}

[[nodiscard]] json::Value op_neural_network(const json::Value& args) {
    const int hidden_units = optional_int(args, "hidden", 14, 2, 64);
    const double learning_rate = optional_scalar(args, "learning_rate", 0.05, 1e-4, 1.0);
    const int epochs = optional_int(args, "epochs", 400, 5, 5000);
    const int samples = optional_int(args, "samples", 400, 32, 4000);
    const int seed = optional_int(args, "seed", 31, kSeedMin, kSeedMax);

    const learning::PlanarSubChain chain = learning::gp8_planar_subchain();
    // The elbow-down branch of the L-U pair, so the inverse map the network has
    // to learn is single-valued. Both ranges sit inside the datasheet travel.
    const double q1_low = -0.6;
    const double q1_high = 1.4;
    const double q2_low = -1.8;
    const double q2_high = -0.3;

    sensing::SeededRng rng(seed);
    const auto n = static_cast<std::size_t>(samples);
    std::vector<std::array<double, 2>> features(n);
    std::vector<std::array<double, 2>> labels(n);
    std::vector<double> reach_values(n);
    std::vector<double> height_values(n);
    std::vector<double> q1_values(n);
    std::vector<double> q2_values(n);

    for (std::size_t i = 0; i < n; ++i) {
        const double q1 = rng.uniform(q1_low, q1_high);
        const double q2 = rng.uniform(q2_low, q2_high);
        const Eigen::Vector2d tip = learning::planar_forward(chain, q1, q2);
        features[i] = {tip.x(), tip.y()};
        labels[i] = {q1, q2};
        reach_values[i] = tip.x();
        height_values[i] = tip.y();
        q1_values[i] = q1;
        q2_values[i] = q2;
    }

    const std::array<Normaliser, 2> input_norm = {fit_normaliser(reach_values),
                                                  fit_normaliser(height_values)};
    const std::array<Normaliser, 2> output_norm = {fit_normaliser(q1_values),
                                                   fit_normaliser(q2_values)};

    // A chronological split: the last fifth is never trained on.
    const std::size_t train_count = (n * 4) / 5;
    const std::size_t test_count = n - train_count;
    if (train_count < 8 || test_count < 2) {
        throw StudyError("parameter 'samples' = " + std::to_string(samples) +
                         " is too small to split into a training and a test set");
    }

    Network net = make_network(2, static_cast<std::size_t>(hidden_units), 2, rng);
    std::vector<double> hidden(net.hidden, 0.0);
    std::vector<double> output(net.outputs, 0.0);
    std::vector<double> delta_output(net.outputs, 0.0);
    std::vector<double> delta_hidden(net.hidden, 0.0);
    std::vector<double> x(2, 0.0);
    std::vector<double> target(2, 0.0);
    std::vector<std::size_t> order(train_count);
    for (std::size_t i = 0; i < train_count; ++i) {
        order[i] = i;
    }

    std::vector<double> epoch_index;
    std::vector<double> train_loss;
    std::vector<double> test_loss;
    epoch_index.reserve(static_cast<std::size_t>(epochs));
    train_loss.reserve(static_cast<std::size_t>(epochs));
    test_loss.reserve(static_cast<std::size_t>(epochs));

    const auto evaluate_loss = [&](std::size_t from, std::size_t to) {
        double sum = 0.0;
        for (std::size_t i = from; i < to; ++i) {
            x[0] = input_norm[0].forward(features[i][0]);
            x[1] = input_norm[1].forward(features[i][1]);
            forward_pass(net, x, hidden, output);
            for (std::size_t o = 0; o < 2; ++o) {
                const double e = output[o] - output_norm[o].forward(labels[i][o]);
                sum += e * e;
            }
        }
        const auto count = static_cast<double>(to - from);
        return (count > 0.0) ? sum / (2.0 * count) : 0.0;
    };

    for (int epoch = 0; epoch < epochs; ++epoch) {
        // Fisher-Yates with the seeded stream: the sample order is part of the
        // reproducible run, not an accident of the container.
        for (std::size_t i = train_count; i > 1; --i) {
            const std::size_t j = rng.index(i);
            std::swap(order[i - 1], order[j]);
        }
        for (const std::size_t index : order) {
            x[0] = input_norm[0].forward(features[index][0]);
            x[1] = input_norm[1].forward(features[index][1]);
            target[0] = output_norm[0].forward(labels[index][0]);
            target[1] = output_norm[1].forward(labels[index][1]);
            forward_pass(net, x, hidden, output);

            // Output layer is linear with a squared error, so its delta is the
            // residual itself.
            for (std::size_t o = 0; o < net.outputs; ++o) {
                delta_output[o] = output[o] - target[o];
            }
            // Hidden layer: the weighted residual times tanh'(z) = 1 - tanh^2.
            for (std::size_t h = 0; h < net.hidden; ++h) {
                double sum = 0.0;
                for (std::size_t o = 0; o < net.outputs; ++o) {
                    sum += net.w2[o * net.hidden + h] * delta_output[o];
                }
                delta_hidden[h] = sum * (1.0 - hidden[h] * hidden[h]);
            }
            for (std::size_t o = 0; o < net.outputs; ++o) {
                for (std::size_t h = 0; h < net.hidden; ++h) {
                    net.w2[o * net.hidden + h] -= learning_rate * delta_output[o] * hidden[h];
                }
                net.b2[o] -= learning_rate * delta_output[o];
            }
            for (std::size_t h = 0; h < net.hidden; ++h) {
                for (std::size_t i = 0; i < net.inputs; ++i) {
                    net.w1[h * net.inputs + i] -= learning_rate * delta_hidden[h] * x[i];
                }
                net.b1[h] -= learning_rate * delta_hidden[h];
            }
        }
        epoch_index.push_back(static_cast<double>(epoch + 1));
        train_loss.push_back(evaluate_loss(0, train_count));
        test_loss.push_back(evaluate_loss(train_count, n));
    }

    // Final error in radians on the held-out set, against the trivial model
    // that always predicts the training mean of each joint.
    std::array<double, 2> training_mean = {0.0, 0.0};
    for (std::size_t i = 0; i < train_count; ++i) {
        training_mean[0] += labels[i][0];
        training_mean[1] += labels[i][1];
    }
    training_mean[0] /= static_cast<double>(train_count);
    training_mean[1] /= static_cast<double>(train_count);

    double network_square_sum = 0.0;
    double baseline_square_sum = 0.0;
    double worst_error = 0.0;
    std::vector<double> test_index;
    std::vector<double> predicted_q1;
    std::vector<double> true_q1;
    std::vector<double> predicted_q2;
    std::vector<double> true_q2;
    test_index.reserve(test_count);
    predicted_q1.reserve(test_count);
    true_q1.reserve(test_count);
    predicted_q2.reserve(test_count);
    true_q2.reserve(test_count);

    for (std::size_t i = train_count; i < n; ++i) {
        x[0] = input_norm[0].forward(features[i][0]);
        x[1] = input_norm[1].forward(features[i][1]);
        forward_pass(net, x, hidden, output);
        const double q1_hat = output_norm[0].inverse(output[0]);
        const double q2_hat = output_norm[1].inverse(output[1]);
        const double e1 = q1_hat - labels[i][0];
        const double e2 = q2_hat - labels[i][1];
        network_square_sum += e1 * e1 + e2 * e2;
        const double b1 = training_mean[0] - labels[i][0];
        const double b2 = training_mean[1] - labels[i][1];
        baseline_square_sum += b1 * b1 + b2 * b2;
        worst_error = std::max(worst_error, std::max(std::abs(e1), std::abs(e2)));
        test_index.push_back(static_cast<double>(i));
        predicted_q1.push_back(q1_hat);
        true_q1.push_back(labels[i][0]);
        predicted_q2.push_back(q2_hat);
        true_q2.push_back(labels[i][1]);
    }

    const double divisor = 2.0 * static_cast<double>(test_count);
    const double final_error = std::sqrt(network_square_sum / divisor);
    const double baseline_error = std::sqrt(baseline_square_sum / divisor);

    json::Value loss_curve = json::Value::array();
    loss_curve.push_back(json::from_series("training loss", epoch_index, train_loss));
    loss_curve.push_back(json::from_series("test loss", epoch_index, test_loss));

    json::Value comparison = json::Value::array();
    comparison.push_back(json::from_series("L axis true [rad]", test_index, true_q1));
    comparison.push_back(json::from_series("L axis learned [rad]", test_index, predicted_q1));
    comparison.push_back(json::from_series("U axis true [rad]", test_index, true_q2));
    comparison.push_back(json::from_series("U axis learned [rad]", test_index, predicted_q2));

    const std::vector<std::vector<json::Value>> architecture_rows = {
        {json::Value("input"), json::Value(2.0), json::Value("linear"),
         json::Value("normalised (reach, height) of the L-U tip")},
        {json::Value("hidden"), json::Value(static_cast<double>(hidden_units)),
         json::Value("tanh"), json::Value("weights initialised by Xavier from the seed")},
        {json::Value("output"), json::Value(2.0), json::Value("linear"),
         json::Value("normalised (q_L, q_U), de-normalised to radians")},
    };

    json::Value out = json::Value::object();
    out.set("loss_curve", std::move(loss_curve));
    out.set("learned_vs_true", std::move(comparison));
    out.set("architecture",
            json::from_table({"layer", "units", "activation", "meaning"}, architecture_rows));
    out.set("final_error", json::Value(final_error));
    out.set("baseline_error", json::Value(baseline_error));
    out.set("improvement_factor",
            json::Value((final_error > 0.0) ? baseline_error / final_error
                                            : std::numeric_limits<double>::infinity()));
    out.set("beats_baseline", json::Value(final_error < baseline_error));
    out.set("worst_test_error", json::Value(worst_error));
    out.set("initial_loss", json::Value(train_loss.empty() ? 0.0 : train_loss.front()));
    out.set("final_loss", json::Value(train_loss.empty() ? 0.0 : train_loss.back()));
    out.set("final_test_loss", json::Value(test_loss.empty() ? 0.0 : test_loss.back()));
    out.set("link_lengths",
            json::from_table({"l1_m", "l2_m"},
                             std::vector<std::vector<double>>{{chain.l1, chain.l2}}));
    out.set("train_samples", json::Value(static_cast<double>(train_count)));
    out.set("test_samples", json::Value(static_cast<double>(test_count)));
    out.set("seed", json::Value(seed));
    out.set("note",
            json::Value("The trivial baseline always predicts the training mean of each joint and "
                        "scores " +
                        json::number_to_string(baseline_error) +
                        " rad RMS on the held-out set; this network scores " +
                        json::number_to_string(final_error) +
                        " rad. The gap is what was learned, and the test-loss curve is where to "
                        "look for over-training: once it turns upward while the training loss "
                        "keeps falling, the extra epochs are memorising samples."));
    return out;
}

// ---------------------------------------------------------------------------
// genetic_algorithm: PID gains for the L-axis drive
// ---------------------------------------------------------------------------

struct PidScore {
    double fitness = 0.0;
    double integral_square_error = 0.0;
    double overshoot = 0.0;
    double effort = 0.0;
    double settling_time = std::numeric_limits<double>::quiet_NaN();
};

// One joint of the real drive train: the reflected rotor inertia plus the
// link's own inertia about the axis, with the datasheet torque ceiling.
struct JointPlant {
    double inertia = 0.0;
    double damping = 0.0;
    double torque_limit = 0.0;
};

[[nodiscard]] JointPlant l_axis_plant() noexcept {
    JointPlant plant;
    const std::size_t joint = 1;  // L axis
    plant.inertia = reflected_rotor_inertia(joint) + link_inertia(joint)(1, 1) +
                    GP8_LINKS[joint].mass * GP8_DH[joint].a * GP8_DH[joint].a;
    plant.damping = GP8_LINKS[joint].viscous_friction;
    plant.torque_limit = GP8_LINKS[joint].max_torque;
    return plant;
}

[[nodiscard]] PidScore score_pid(const JointPlant& plant, double kp, double ki, double kd,
                                 double setpoint, double dt, int steps) noexcept {
    PidScore score;
    double q = 0.0;
    double qdot = 0.0;
    double integral = 0.0;
    double previous_error = setpoint;
    double peak = 0.0;
    bool settled = false;
    const double band = 0.02 * std::abs(setpoint);

    for (int k = 0; k < steps; ++k) {
        const double error = setpoint - q;
        integral += error * dt;
        const double derivative = (error - previous_error) / dt;
        previous_error = error;
        double torque = kp * error + ki * integral + kd * derivative;
        torque = std::clamp(torque, -plant.torque_limit, plant.torque_limit);
        const double acceleration = (torque - plant.damping * qdot) / plant.inertia;
        qdot += acceleration * dt;
        q += qdot * dt;

        score.integral_square_error += error * error * dt;
        score.effort += torque * torque * dt;
        peak = std::max(peak, q);
        if (std::abs(error) > band) {
            settled = false;
        } else if (!settled) {
            settled = true;
            score.settling_time = dt * static_cast<double>(k);
        }
    }
    score.overshoot = std::max(0.0, (peak - setpoint) / std::abs(setpoint));
    if (!std::isfinite(score.integral_square_error) || !std::isfinite(score.effort)) {
        score.fitness = 0.0;
        return score;
    }
    // Maximised, bounded in (0, 1]: a clean fast response scores near 1, an
    // unstable one scores near 0. Overshoot is penalised because a welding
    // torch that overshoots has already touched the workpiece.
    score.fitness = 1.0 / (1.0 + 10.0 * score.integral_square_error + 5.0 * score.overshoot +
                           1e-5 * score.effort);
    return score;
}

[[nodiscard]] json::Value op_genetic_algorithm(const json::Value& args) {
    const int population_size = optional_int(args, "population", 40, 4, 200);
    const int generations = optional_int(args, "generations", 30, 2, 200);
    const double mutation_rate = optional_scalar(args, "mutation_rate", 0.25, 0.0, 1.0);
    const double mutation_sigma = optional_scalar(args, "mutation_sigma", 0.12, 0.0, 1.0);
    const double crossover_rate = optional_scalar(args, "crossover_rate", 0.8, 0.0, 1.0);
    const bool elitism = optional_bool(args, "elitism", true);
    const int tournament = optional_int(args, "tournament", 3, 2, 16);
    const double setpoint = optional_scalar(args, "setpoint", 0.5, 0.01, 1.5);
    const int seed = optional_int(args, "seed", 41, kSeedMin, kSeedMax);

    const JointPlant plant = l_axis_plant();
    const double dt = 0.002;
    const int steps = 750;  // 1.5 s of step response

    // Genes are normalised to [0, 1] so mutation has one meaningful sigma.
    const std::array<double, 3> gene_low = {0.0, 0.0, 0.0};
    const std::array<double, 3> gene_high = {400.0, 600.0, 40.0};
    const std::array<const char*, 3> gene_names = {"Kp", "Ki", "Kd"};
    const auto decode = [&](const std::array<double, 3>& genome, std::size_t g) {
        return gene_low[g] + (gene_high[g] - gene_low[g]) * genome[g];
    };

    sensing::SeededRng rng(seed);
    const auto pop = static_cast<std::size_t>(population_size);
    std::vector<std::array<double, 3>> individuals(pop);
    std::vector<double> fitness(pop, 0.0);
    for (auto& genome : individuals) {
        for (std::size_t g = 0; g < 3; ++g) {
            genome[g] = rng.uniform01();
        }
    }

    const auto evaluate = [&](const std::array<double, 3>& genome) {
        return score_pid(plant, decode(genome, 0), decode(genome, 1), decode(genome, 2), setpoint,
                         dt, steps);
    };

    std::vector<double> generation_index;
    std::vector<double> best_history;
    std::vector<double> mean_history;
    std::vector<double> diversity_history;
    generation_index.reserve(static_cast<std::size_t>(generations));
    best_history.reserve(static_cast<std::size_t>(generations));
    mean_history.reserve(static_cast<std::size_t>(generations));
    diversity_history.reserve(static_cast<std::size_t>(generations));

    std::array<double, 3> best_genome = individuals.front();
    double best_fitness = -1.0;

    for (int generation = 0; generation < generations; ++generation) {
        double sum = 0.0;
        std::size_t best_index = 0;
        for (std::size_t i = 0; i < pop; ++i) {
            fitness[i] = evaluate(individuals[i]).fitness;
            sum += fitness[i];
            if (fitness[i] > fitness[best_index]) {
                best_index = i;
            }
        }
        if (fitness[best_index] > best_fitness) {
            best_fitness = fitness[best_index];
            best_genome = individuals[best_index];
        }

        // Diversity: the mean per-gene standard deviation of the population in
        // normalised gene space. Selection pressure drives it to zero, and
        // that collapse is what the student is meant to see.
        double diversity = 0.0;
        for (std::size_t g = 0; g < 3; ++g) {
            double gene_mean = 0.0;
            for (const auto& genome : individuals) {
                gene_mean += genome[g];
            }
            gene_mean /= static_cast<double>(pop);
            double variance = 0.0;
            for (const auto& genome : individuals) {
                const double d = genome[g] - gene_mean;
                variance += d * d;
            }
            diversity += std::sqrt(variance / static_cast<double>(pop));
        }
        diversity /= 3.0;

        generation_index.push_back(static_cast<double>(generation + 1));
        best_history.push_back(best_fitness);
        mean_history.push_back(sum / static_cast<double>(pop));
        diversity_history.push_back(diversity);

        if (generation + 1 == generations) {
            break;
        }

        std::vector<std::array<double, 3>> next;
        next.reserve(pop);
        if (elitism) {
            // The elite is copied unchanged, which is exactly why the
            // best-so-far curve can never step downward.
            next.push_back(individuals[best_index]);
        }
        const auto tournament_pick = [&]() -> const std::array<double, 3>& {
            std::size_t winner = rng.index(pop);
            for (int t = 1; t < tournament; ++t) {
                const std::size_t challenger = rng.index(pop);
                if (fitness[challenger] > fitness[winner]) {
                    winner = challenger;
                }
            }
            return individuals[winner];
        };
        while (next.size() < pop) {
            const std::array<double, 3> parent_a = tournament_pick();
            const std::array<double, 3> parent_b = tournament_pick();
            std::array<double, 3> child = parent_a;
            if (rng.uniform01() < crossover_rate) {
                // Blend crossover: a child anywhere on the segment between the
                // parents, which keeps the search continuous.
                const double blend = rng.uniform01();
                for (std::size_t g = 0; g < 3; ++g) {
                    child[g] = blend * parent_a[g] + (1.0 - blend) * parent_b[g];
                }
            }
            for (std::size_t g = 0; g < 3; ++g) {
                if (rng.uniform01() < mutation_rate) {
                    child[g] = std::clamp(child[g] + mutation_sigma * rng.gaussian(), 0.0, 1.0);
                }
            }
            next.push_back(child);
        }
        individuals = std::move(next);
    }

    const PidScore best = evaluate(best_genome);

    // The step response of the winner, so the fitness number has a picture.
    std::vector<double> time;
    std::vector<double> response;
    std::vector<double> reference;
    time.reserve(static_cast<std::size_t>(steps));
    response.reserve(static_cast<std::size_t>(steps));
    reference.reserve(static_cast<std::size_t>(steps));
    {
        double q = 0.0;
        double qdot = 0.0;
        double integral = 0.0;
        double previous_error = setpoint;
        for (int k = 0; k < steps; ++k) {
            const double error = setpoint - q;
            integral += error * dt;
            const double derivative = (error - previous_error) / dt;
            previous_error = error;
            double torque = decode(best_genome, 0) * error + decode(best_genome, 1) * integral +
                            decode(best_genome, 2) * derivative;
            torque = std::clamp(torque, -plant.torque_limit, plant.torque_limit);
            const double acceleration = (torque - plant.damping * qdot) / plant.inertia;
            qdot += acceleration * dt;
            q += qdot * dt;
            time.push_back(dt * static_cast<double>(k));
            response.push_back(q);
            reference.push_back(setpoint);
        }
    }

    json::Value curves = json::Value::array();
    curves.push_back(json::from_series("best fitness", generation_index, best_history));
    curves.push_back(json::from_series("mean fitness", generation_index, mean_history));
    curves.push_back(json::from_series("gene diversity", generation_index, diversity_history));

    json::Value step_response = json::Value::array();
    step_response.push_back(json::from_series("q [rad]", time, response));
    step_response.push_back(json::from_series("setpoint [rad]", time, reference));

    std::vector<std::vector<json::Value>> genome_rows;
    genome_rows.reserve(3);
    for (std::size_t g = 0; g < 3; ++g) {
        genome_rows.push_back({json::Value(gene_names[g]), json::Value(best_genome[g]),
                               json::Value(decode(best_genome, g)), json::Value(gene_low[g]),
                               json::Value(gene_high[g])});
    }

    bool monotone = true;
    for (std::size_t i = 1; i < best_history.size(); ++i) {
        if (best_history[i] < best_history[i - 1]) {
            monotone = false;
            break;
        }
    }

    json::Value out = json::Value::object();
    out.set("curves", std::move(curves));
    out.set("step_response", std::move(step_response));
    out.set("best_genome",
            json::from_table({"gene", "normalised", "value", "min", "max"}, genome_rows));
    out.set("best_fitness", json::Value(best_fitness));
    out.set("best_fitness_history", json::from_series("best fitness", generation_index,
                                                      best_history));
    out.set("diversity_initial",
            json::Value(diversity_history.empty() ? 0.0 : diversity_history.front()));
    out.set("diversity_final",
            json::Value(diversity_history.empty() ? 0.0 : diversity_history.back()));
    out.set("best_fitness_monotone", json::Value(monotone));
    out.set("integral_square_error", json::Value(best.integral_square_error));
    out.set("overshoot", json::Value(best.overshoot));
    out.set("settling_time", json::Value(best.settling_time));
    out.set("plant",
            json::from_table({"inertia_kgm2", "damping_Nms", "torque_limit_Nm"},
                             std::vector<std::vector<double>>{
                                 {plant.inertia, plant.damping, plant.torque_limit}}));
    out.set("elitism", json::Value(elitism));
    out.set("seed", json::Value(seed));
    out.set("note",
            json::Value("With elitism on, the best individual is copied into the next generation "
                        "unchanged, so the best-so-far curve is monotonically non-decreasing by "
                        "construction - not by luck. Set the mutation rate to zero and the "
                        "diversity collapses within a few generations and the search stalls "
                        "wherever it happened to be; restore it and the population escapes. "
                        "Selection, crossover and mutation each do the job the lecture assigns "
                        "them, and the diversity curve is the evidence."));
    return out;
}

// ---------------------------------------------------------------------------
// decision_tree: will this motion violate a limit?
// ---------------------------------------------------------------------------

struct Sample {
    std::array<double, 4> features{};
    bool label = false;
};

struct TreeNode {
    std::size_t id = 0;
    int depth = 0;
    std::size_t feature = 0;
    double threshold = 0.0;
    double information_gain = 0.0;
    std::size_t samples = 0;
    std::size_t positives = 0;
    bool leaf = true;
    bool prediction = false;
    std::size_t left = 0;
    std::size_t right = 0;
};

// The label is computed from the robot, not invented: a motion is a violation
// when it would drive a joint past its datasheet travel inside the horizon, or
// when the payload's static moment exceeds the L-axis torque ceiling.
[[nodiscard]] bool motion_violates_limits(const std::array<double, 4>& f, double horizon) noexcept {
    const double q_l = f[0];
    const double q_u = f[1];
    const double speed_scale = f[2];
    const double payload_fraction = f[3];

    const double next_l = q_l + speed_scale * joint_max_velocity(1) * horizon;
    const double next_u = q_u + speed_scale * joint_max_velocity(2) * horizon;
    if (next_l > joint_max(1) || next_u > joint_max(2)) {
        return true;
    }
    const double reach = std::abs(GP8_DH[1].a * std::cos(q_l)) + std::abs(GP8_DH[3].d);
    const double static_torque =
        payload_fraction * GP8_PAYLOAD_KG * GP8_GRAVITY_MPS2 * reach +
        GP8_LINKS[1].mass * GP8_GRAVITY_MPS2 * 0.5 * GP8_DH[1].a;
    return static_torque > GP8_LINKS[1].max_torque;
}

struct SplitChoice {
    bool found = false;
    std::size_t feature = 0;
    double threshold = 0.0;
    double gain = 0.0;
};

[[nodiscard]] SplitChoice best_split(const std::vector<Sample>& data,
                                     const std::vector<std::size_t>& indices,
                                     std::size_t min_leaf, int candidate_thresholds) {
    SplitChoice best;
    const std::size_t total = indices.size();
    std::size_t positives = 0;
    for (const std::size_t i : indices) {
        positives += data[i].label ? 1u : 0u;
    }
    const double parent_entropy = learning::binary_entropy(positives, total);
    if (parent_entropy <= 0.0) {
        return best;
    }

    for (std::size_t f = 0; f < 4; ++f) {
        double low = data[indices.front()].features[f];
        double high = low;
        for (const std::size_t i : indices) {
            low = std::min(low, data[i].features[f]);
            high = std::max(high, data[i].features[f]);
        }
        if (high - low < 1e-12) {
            continue;
        }
        for (int c = 1; c <= candidate_thresholds; ++c) {
            const double threshold =
                low + (high - low) * static_cast<double>(c) /
                          static_cast<double>(candidate_thresholds + 1);
            std::size_t left_total = 0;
            std::size_t left_positives = 0;
            for (const std::size_t i : indices) {
                if (data[i].features[f] <= threshold) {
                    ++left_total;
                    left_positives += data[i].label ? 1u : 0u;
                }
            }
            const std::size_t right_total = total - left_total;
            if (left_total < min_leaf || right_total < min_leaf) {
                continue;
            }
            const double left_weight = static_cast<double>(left_total) /
                                       static_cast<double>(total);
            const double right_weight = 1.0 - left_weight;
            const double gain =
                parent_entropy -
                left_weight * learning::binary_entropy(left_positives, left_total) -
                right_weight *
                    learning::binary_entropy(positives - left_positives, right_total);
            if (gain > best.gain + 1e-15) {
                best.found = true;
                best.gain = gain;
                best.feature = f;
                best.threshold = threshold;
            }
        }
    }
    return best;
}

void grow_tree(const std::vector<Sample>& data, std::vector<std::size_t> indices, int depth,
               int max_depth, std::size_t min_split, std::size_t min_leaf,
               int candidate_thresholds, std::vector<TreeNode>& nodes, std::size_t node_id) {
    TreeNode& node = nodes[node_id];
    node.id = node_id;
    node.depth = depth;
    node.samples = indices.size();
    node.positives = 0;
    for (const std::size_t i : indices) {
        node.positives += data[i].label ? 1u : 0u;
    }
    node.prediction = (2 * node.positives >= node.samples) && node.samples > 0;
    node.leaf = true;

    if (depth >= max_depth || indices.size() < min_split || node.positives == 0 ||
        node.positives == node.samples) {
        return;
    }
    const SplitChoice split = best_split(data, indices, min_leaf, candidate_thresholds);
    if (!split.found) {
        return;
    }

    std::vector<std::size_t> left;
    std::vector<std::size_t> right;
    left.reserve(indices.size());
    right.reserve(indices.size());
    for (const std::size_t i : indices) {
        if (data[i].features[split.feature] <= split.threshold) {
            left.push_back(i);
        } else {
            right.push_back(i);
        }
    }

    node.leaf = false;
    node.feature = split.feature;
    node.threshold = split.threshold;
    node.information_gain = split.gain;
    nodes.emplace_back();
    const std::size_t left_id = nodes.size() - 1;
    nodes.emplace_back();
    const std::size_t right_id = nodes.size() - 1;
    nodes[node_id].left = left_id;
    nodes[node_id].right = right_id;

    grow_tree(data, std::move(left), depth + 1, max_depth, min_split, min_leaf,
              candidate_thresholds, nodes, left_id);
    grow_tree(data, std::move(right), depth + 1, max_depth, min_split, min_leaf,
              candidate_thresholds, nodes, right_id);
}

[[nodiscard]] bool classify(const std::vector<TreeNode>& nodes, const std::array<double, 4>& f) {
    std::size_t node_id = 0;
    for (int guard = 0; guard < 256; ++guard) {
        const TreeNode& node = nodes[node_id];
        if (node.leaf) {
            return node.prediction;
        }
        node_id = (f[node.feature] <= node.threshold) ? node.left : node.right;
    }
    return nodes[0].prediction;
}

[[nodiscard]] json::Value op_decision_tree(const json::Value& args) {
    const int samples = optional_int(args, "samples", 600, 40, 5000);
    const int max_depth = optional_int(args, "max_depth", 4, 1, 10);
    const int min_split = optional_int(args, "min_samples_split", 12, 2, 500);
    const int min_leaf = optional_int(args, "min_samples_leaf", 4, 1, 250);
    const int candidate_thresholds = optional_int(args, "candidate_thresholds", 15, 3, 63);
    const double horizon = optional_scalar(args, "horizon", 0.35, 0.01, 2.0);
    const int seed = optional_int(args, "seed", 53, kSeedMin, kSeedMax);

    const std::array<const char*, 4> feature_names = {"q_L [rad]", "q_U [rad]", "speed_scale",
                                                      "payload_fraction"};

    sensing::SeededRng rng(seed);
    const auto n = static_cast<std::size_t>(samples);
    std::vector<Sample> data(n);
    std::size_t positives = 0;
    for (std::size_t i = 0; i < n; ++i) {
        Sample s;
        s.features[0] = rng.uniform(joint_min(1), joint_max(1));
        s.features[1] = rng.uniform(joint_min(2), joint_max(2));
        s.features[2] = rng.uniform(0.05, 1.0);
        s.features[3] = rng.uniform(0.0, 1.2);
        s.label = motion_violates_limits(s.features, horizon);
        positives += s.label ? 1u : 0u;
        data[i] = s;
    }

    const std::size_t train_count = (n * 7) / 10;
    std::vector<std::size_t> train_indices;
    train_indices.reserve(train_count);
    for (std::size_t i = 0; i < train_count; ++i) {
        train_indices.push_back(i);
    }

    std::vector<TreeNode> nodes;
    nodes.reserve(2 * static_cast<std::size_t>(1 << std::min(max_depth, 10)));
    nodes.emplace_back();
    grow_tree(data, train_indices, 0, max_depth, static_cast<std::size_t>(min_split),
              static_cast<std::size_t>(min_leaf), candidate_thresholds, nodes, 0);

    std::size_t train_correct = 0;
    for (std::size_t i = 0; i < train_count; ++i) {
        train_correct += (classify(nodes, data[i].features) == data[i].label) ? 1u : 0u;
    }
    std::size_t test_correct = 0;
    std::size_t true_positive = 0;
    std::size_t false_positive = 0;
    std::size_t false_negative = 0;
    for (std::size_t i = train_count; i < n; ++i) {
        const bool predicted = classify(nodes, data[i].features);
        test_correct += (predicted == data[i].label) ? 1u : 0u;
        if (predicted && data[i].label) {
            ++true_positive;
        } else if (predicted && !data[i].label) {
            ++false_positive;
        } else if (!predicted && data[i].label) {
            ++false_negative;
        }
    }
    const std::size_t test_count = n - train_count;

    std::vector<std::vector<json::Value>> rule_rows;
    rule_rows.reserve(nodes.size());
    std::vector<double> gain_x;
    std::vector<double> gain_y;
    std::size_t leaf_count = 0;
    for (const TreeNode& node : nodes) {
        const std::string rule =
            node.leaf ? (std::string("predict ") + (node.prediction ? "VIOLATION" : "safe"))
                      : (std::string(feature_names[node.feature]) + " <= " +
                         json::number_to_string(node.threshold));
        rule_rows.push_back({json::Value(static_cast<double>(node.id)),
                             json::Value(static_cast<double>(node.depth)), json::Value(rule),
                             json::Value(node.information_gain),
                             json::Value(static_cast<double>(node.samples)),
                             json::Value(static_cast<double>(node.positives)),
                             json::Value(learning::binary_entropy(node.positives, node.samples)),
                             json::Value(node.leaf)});
        if (!node.leaf) {
            gain_x.push_back(static_cast<double>(node.id));
            gain_y.push_back(node.information_gain);
        } else {
            ++leaf_count;
        }
    }

    json::Value out = json::Value::object();
    out.set("tree", json::from_table({"node", "depth", "rule", "information_gain_bits",
                                      "samples", "violations", "entropy_bits", "is_leaf"},
                                     rule_rows));
    out.set("information_gain", json::from_series("information gain per split [bits]", gain_x,
                                                  gain_y));
    out.set("features", json::from_strings({feature_names[0], feature_names[1], feature_names[2],
                                            feature_names[3]}));
    out.set("train_accuracy",
            json::Value(static_cast<double>(train_correct) / static_cast<double>(train_count)));
    out.set("test_accuracy",
            json::Value(static_cast<double>(test_correct) / static_cast<double>(test_count)));
    out.set("majority_baseline_accuracy",
            json::Value(std::max(static_cast<double>(positives),
                                 static_cast<double>(n - positives)) /
                        static_cast<double>(n)));
    out.set("confusion",
            json::from_table({"true_positive", "false_positive", "false_negative",
                              "true_negative"},
                             std::vector<std::vector<double>>{
                                 {static_cast<double>(true_positive),
                                  static_cast<double>(false_positive),
                                  static_cast<double>(false_negative),
                                  static_cast<double>(test_count - true_positive -
                                                      false_positive - false_negative)}}));
    out.set("node_count", json::Value(static_cast<double>(nodes.size())));
    out.set("leaf_count", json::Value(static_cast<double>(leaf_count)));
    out.set("violation_rate",
            json::Value(static_cast<double>(positives) / static_cast<double>(n)));
    out.set("root_entropy", json::Value(learning::binary_entropy(positives, n)));
    out.set("seed", json::Value(seed));
    out.set("note",
            json::Value("Information gain is the parent's entropy minus the weighted entropy of "
                        "the two children, in bits, and the split with the largest gain wins. "
                        "The reason to choose a tree here rather than a better classifier is "
                        "that every rule is readable: a leaf that says 'speed_scale above 0.7 and "
                        "q_L above 1.9 is a violation' can be checked against the datasheet by "
                        "hand. A false negative here is a robot driven into its own limit, so "
                        "the confusion table matters more than the accuracy."));
    return out;
}

// ---------------------------------------------------------------------------
// agent_policy: three architectures from the course on one pick-and-place task
// ---------------------------------------------------------------------------

struct Part {
    double reach = 0.0;       // radial distance in the tray [m]
    bool misoriented = false; // needs a reorientation before it can be gripped
    double mass = 0.0;        // [kg]
};

struct TraceRow {
    int step = 0;
    std::string percept;
    std::string state;
    std::string action;
    std::string reason;
    double cost = 0.0;
};

struct AgentResult {
    std::vector<std::string> actions;
    std::vector<TraceRow> trace;
    double score = 0.0;
    double time_cost = 0.0;
    std::size_t placed = 0;
    std::size_t failed_attempts = 0;
};

constexpr double kPickCost = 1.2;
constexpr double kPlaceCost = 1.0;
constexpr double kReorientCost = 0.8;
constexpr double kFailedPickCost = 1.5;  // a failed grip still costs the move
constexpr double kPlaceReward = 10.0;

[[nodiscard]] std::string part_label(std::size_t index) {
    return "part " + std::to_string(index);
}

// A simple reflex agent maps the CURRENT percept to an action and keeps no
// state, so it cannot learn from a grip it has already failed.
[[nodiscard]] AgentResult run_reflex_agent(const std::vector<Part>& parts) {
    AgentResult result;
    int step = 0;
    for (std::size_t i = 0; i < parts.size(); ++i) {
        const Part& part = parts[i];
        const std::string percept =
            part_label(i) + (part.misoriented ? " looks rotated" : " looks aligned");
        result.actions.push_back("pick " + part_label(i));
        if (part.misoriented) {
            result.time_cost += kFailedPickCost;
            ++result.failed_attempts;
            result.trace.push_back({step++, percept, "no internal state",
                                    "pick " + part_label(i),
                                    "the rule says pick whatever is in front of the gripper",
                                    kFailedPickCost});
            continue;
        }
        result.time_cost += kPickCost + kPlaceCost;
        ++result.placed;
        result.trace.push_back({step++, percept, "no internal state",
                                "pick and place " + part_label(i),
                                "the rule fires and the grip happens to succeed",
                                kPickCost + kPlaceCost});
    }
    result.score = kPlaceReward * static_cast<double>(result.placed) - result.time_cost;
    return result;
}

// A model-based agent keeps an internal model of the world, so an observed
// misorientation becomes a reorientation instead of a failed grip.
[[nodiscard]] AgentResult run_model_based_agent(const std::vector<Part>& parts) {
    AgentResult result;
    int step = 0;
    std::size_t known_misoriented = 0;
    for (std::size_t i = 0; i < parts.size(); ++i) {
        const Part& part = parts[i];
        const std::string percept =
            part_label(i) + (part.misoriented ? " looks rotated" : " looks aligned");
        if (part.misoriented) {
            ++known_misoriented;
            result.actions.push_back("reorient " + part_label(i));
            result.time_cost += kReorientCost;
            result.trace.push_back({step++, percept,
                                    "model holds " + std::to_string(known_misoriented) +
                                        " rotated parts",
                                    "reorient " + part_label(i),
                                    "the model says a rotated part cannot be gripped, so fix the "
                                    "orientation first",
                                    kReorientCost});
        }
        result.actions.push_back("pick " + part_label(i));
        result.time_cost += kPickCost + kPlaceCost;
        ++result.placed;
        result.trace.push_back({step++, percept,
                                "model holds " + std::to_string(known_misoriented) +
                                    " rotated parts",
                                "pick and place " + part_label(i),
                                "the model predicts the grip will now succeed",
                                kPickCost + kPlaceCost});
    }
    result.score = kPlaceReward * static_cast<double>(result.placed) - result.time_cost;
    return result;
}

// A goal-based utility agent has the same model plus a goal (every part
// placed) and a utility it maximises, so it orders the work by utility rather
// than by tray position.
[[nodiscard]] AgentResult run_goal_based_agent(const std::vector<Part>& parts) {
    AgentResult result;
    int step = 0;
    std::vector<bool> done(parts.size(), false);

    for (std::size_t placed = 0; placed < parts.size(); ++placed) {
        std::size_t choice = parts.size();
        double best_utility = -std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < parts.size(); ++i) {
            if (done[i]) {
                continue;
            }
            // Utility = reward over expected cost. A near, aligned, light part
            // is cheap, so it is worth more per second of cycle time.
            const double cost = kPickCost + kPlaceCost + 0.9 * parts[i].reach +
                                0.25 * parts[i].mass +
                                (parts[i].misoriented ? kReorientCost : 0.0);
            const double utility = kPlaceReward / cost;
            if (utility > best_utility) {
                best_utility = utility;
                choice = i;
            }
        }
        if (choice == parts.size()) {
            break;
        }
        done[choice] = true;
        const Part& part = parts[choice];
        double cost = kPickCost + kPlaceCost;
        if (part.misoriented) {
            cost += kReorientCost;
            result.actions.push_back("reorient " + part_label(choice));
        }
        result.actions.push_back("pick " + part_label(choice));
        result.time_cost += cost;
        ++result.placed;
        result.trace.push_back(
            {step++, part_label(choice) + " at " + json::number_to_string(part.reach) + " m",
             "goal: all " + std::to_string(parts.size()) + " placed, " +
                 std::to_string(parts.size() - placed - 1) + " left",
             (part.misoriented ? "reorient then pick " : "pick ") + part_label(choice),
             "highest utility " + json::number_to_string(best_utility) + " = reward / cost",
             cost});
    }
    result.score = kPlaceReward * static_cast<double>(result.placed) - result.time_cost;
    return result;
}

[[nodiscard]] json::Value op_agent_policy(const json::Value& args) {
    const std::string architecture =
        optional_enum(args, "architecture", "all",
                      {"all", "simple_reflex", "model_based", "goal_based_utility"});
    const int part_count = optional_int(args, "parts", 6, 1, 24);
    const double misorientation_rate =
        optional_scalar(args, "misorientation_rate", 0.4, 0.0, 1.0);
    const int seed = optional_int(args, "seed", 61, kSeedMin, kSeedMax);

    // One world, built once from the seed, and every architecture is then run
    // on exactly that world - which is the only way the comparison means
    // anything.
    sensing::SeededRng rng(seed);
    std::vector<Part> parts;
    parts.reserve(static_cast<std::size_t>(part_count));
    for (int i = 0; i < part_count; ++i) {
        Part part;
        part.reach = rng.uniform(0.25, GP8_HORIZONTAL_REACH_M);
        part.misoriented = rng.uniform01() < misorientation_rate;
        part.mass = rng.uniform(0.2, GP8_PAYLOAD_KG);
        parts.push_back(part);
    }

    const std::array<std::pair<const char*, AgentResult>, 3> runs = {{
        {"simple_reflex", run_reflex_agent(parts)},
        {"model_based", run_model_based_agent(parts)},
        {"goal_based_utility", run_goal_based_agent(parts)},
    }};

    std::vector<std::vector<json::Value>> world_rows;
    world_rows.reserve(parts.size());
    for (std::size_t i = 0; i < parts.size(); ++i) {
        world_rows.push_back({json::Value(static_cast<double>(i)), json::Value(parts[i].reach),
                              json::Value(parts[i].mass), json::Value(parts[i].misoriented)});
    }

    std::vector<std::vector<json::Value>> score_rows;
    std::vector<std::vector<json::Value>> trace_rows;
    std::vector<std::vector<json::Value>> action_rows;
    double best_score = -std::numeric_limits<double>::infinity();
    std::string best_architecture;

    for (const auto& run : runs) {
        if (architecture != "all" && architecture != run.first) {
            continue;
        }
        const AgentResult& result = run.second;
        score_rows.push_back({json::Value(run.first),
                              json::Value(static_cast<double>(result.placed)),
                              json::Value(static_cast<double>(result.failed_attempts)),
                              json::Value(result.time_cost), json::Value(result.score)});
        for (std::size_t a = 0; a < result.actions.size(); ++a) {
            action_rows.push_back({json::Value(run.first),
                                   json::Value(static_cast<double>(a)),
                                   json::Value(result.actions[a])});
        }
        for (const TraceRow& row : result.trace) {
            trace_rows.push_back({json::Value(run.first),
                                  json::Value(static_cast<double>(row.step)),
                                  json::Value(row.percept), json::Value(row.state),
                                  json::Value(row.action), json::Value(row.reason),
                                  json::Value(row.cost)});
        }
        if (result.score > best_score) {
            best_score = result.score;
            best_architecture = run.first;
        }
    }

    json::Value out = json::Value::object();
    out.set("world", json::from_table({"part", "reach_m", "mass_kg", "misoriented"}, world_rows));
    out.set("scores", json::from_table({"architecture", "placed", "failed_grips", "time_cost",
                                        "score"},
                                       score_rows));
    out.set("action_sequences",
            json::from_table({"architecture", "order", "action"}, action_rows));
    out.set("decision_trace",
            json::from_table({"architecture", "step", "percept", "internal_state", "action",
                              "reason", "cost"},
                             trace_rows));
    out.set("best_architecture", json::Value(best_architecture));
    out.set("best_score", json::Value(best_score));
    out.set("architecture", json::Value(architecture));
    out.set("parts", json::Value(part_count));
    out.set("seed", json::Value(seed));
    out.set("note",
            json::Value("All three agents see the identical tray. The reflex agent maps the "
                        "current percept straight to an action and keeps nothing, so every "
                        "rotated part costs it a failed grip it cannot learn from. The "
                        "model-based agent carries internal state and converts that percept into "
                        "a reorientation first. The goal-based agent adds a goal and a utility, "
                        "so it also chooses the ORDER of the work - which is the only place a "
                        "utility can show its value on a task where the model-based agent "
                        "already never fails."));
    return out;
}

// ---------------------------------------------------------------------------
// responsible_ai: the audit trail
// ---------------------------------------------------------------------------

struct CheckRecord {
    std::string name;
    std::string category;
    double proposed = 0.0;
    double limit = 0.0;
    std::string unit;
    bool passed = false;
    bool overridable = false;
    std::string reason;
};

[[nodiscard]] json::Value op_responsible_ai(const json::Value& args) {
    const Eigen::Matrix<double, 6, 1> q_current =
        optional_vec6(args, "q_current", Eigen::Matrix<double, 6, 1>::Zero(),
                      -widest_joint_range(), widest_joint_range());
    Eigen::Matrix<double, 6, 1> proposed_default = Eigen::Matrix<double, 6, 1>::Zero();
    proposed_default(1) = 0.9;
    proposed_default(2) = -0.5;
    const Eigen::Matrix<double, 6, 1> q_proposed =
        optional_vec6(args, "q_proposed", proposed_default, -widest_joint_range(),
                      widest_joint_range());
    const double duration = optional_scalar(args, "duration", 1.0, 0.01, 60.0);
    const double payload = optional_scalar(args, "payload", 3.0, 0.0, 40.0);
    const std::string source =
        optional_enum(args, "source", "neural_network",
                      {"neural_network", "genetic_algorithm", "decision_tree", "agent_policy",
                       "human_operator"});
    const bool allow_overrides = optional_bool(args, "allow_overrides", false);
    const double collaborative_speed_limit =
        optional_scalar(args, "collaborative_speed_limit", 0.25, 0.01, 5.0);
    const double minimum_height = optional_scalar(args, "minimum_height", 0.20, 0.0, 2.0);

    std::vector<CheckRecord> checks;

    // --- non-overridable: the machine's own hard limits -------------------
    for (std::size_t j = 0; j < GP8_DOF; ++j) {
        const auto index = static_cast<Eigen::Index>(j);
        const double value = q_proposed(index);
        const bool within = value >= joint_min(j) && value <= joint_max(j);
        const double nearest_limit = (value > joint_max(j)) ? joint_max(j) : joint_min(j);
        checks.push_back({std::string("joint ") + GP8_AXIS_NAMES[j] + " travel", "hard limit",
                          value, within ? joint_max(j) : nearest_limit, "rad", within, false,
                          within ? "inside the datasheet travel"
                                 : "the datasheet travel of this axis would be exceeded; no "
                                   "operator may wave this through"});
    }
    for (std::size_t j = 0; j < GP8_DOF; ++j) {
        const auto index = static_cast<Eigen::Index>(j);
        const double speed = std::abs(q_proposed(index) - q_current(index)) / duration;
        const bool within = speed <= joint_max_velocity(j);
        checks.push_back({std::string("joint ") + GP8_AXIS_NAMES[j] + " speed", "hard limit",
                          speed, joint_max_velocity(j), "rad/s", within, false,
                          within ? "inside the datasheet maximum speed"
                                 : "the commanded move needs more speed than this axis has; "
                                   "shorten it or lengthen the duration"});
    }
    checks.push_back({"rated payload", "hard limit", payload, GP8_PAYLOAD_KG, "kg",
                      payload <= GP8_PAYLOAD_KG, false,
                      payload <= GP8_PAYLOAD_KG
                          ? "inside the rated payload"
                          : "above the rated payload: the torque figures this layer reports "
                            "stop being valid"});

    const Eigen::Isometry3d proposed_pose = forward_kinematics_dh(q_proposed);
    const double horizontal_reach = proposed_pose.translation().head<2>().norm();
    checks.push_back({"horizontal reach", "hard limit", horizontal_reach,
                      GP8_HORIZONTAL_REACH_M, "m", horizontal_reach <= GP8_HORIZONTAL_REACH_M,
                      false,
                      horizontal_reach <= GP8_HORIZONTAL_REACH_M
                          ? "inside the datasheet working envelope"
                          : "outside the working envelope, so the pose is not reachable at all"});

    // --- overridable: cell policy, not machine physics -------------------
    const double tool_speed =
        (proposed_pose.translation() - forward_kinematics_dh(q_current).translation()).norm() /
        duration;
    checks.push_back({"collaborative tool speed", "cell policy", tool_speed,
                      collaborative_speed_limit, "m/s", tool_speed <= collaborative_speed_limit,
                      true,
                      tool_speed <= collaborative_speed_limit
                          ? "below the collaborative speed the cell is configured for"
                          : "above the collaborative speed limit; permissible only behind a "
                            "closed guard, which is a human decision"});
    checks.push_back({"flange height", "cell policy", proposed_pose.translation().z(),
                      minimum_height, "m", proposed_pose.translation().z() >= minimum_height,
                      true,
                      proposed_pose.translation().z() >= minimum_height
                          ? "clear of the table"
                          : "below the configured table clearance; legitimate for a reach into a "
                            "fixture, so a human may accept it"});

    const double smallest_increment =
        sensing::joint_increment(17, GP8_LINKS[1].gear_ratio);
    double smallest_commanded = std::numeric_limits<double>::infinity();
    for (std::size_t j = 0; j < GP8_DOF; ++j) {
        const auto index = static_cast<Eigen::Index>(j);
        const double delta = std::abs(q_proposed(index) - q_current(index));
        if (delta > 0.0) {
            smallest_commanded = std::min(smallest_commanded, delta);
        }
    }
    const bool resolvable = !std::isfinite(smallest_commanded) ||
                            smallest_commanded >= smallest_increment;
    checks.push_back({"commanded increment against encoder resolution", "cell policy",
                      std::isfinite(smallest_commanded) ? smallest_commanded : 0.0,
                      smallest_increment, "rad", resolvable, true,
                      resolvable ? "the smallest commanded step is resolvable by the encoder"
                                 : "the smallest commanded step is below one encoder increment, "
                                   "so the robot cannot verify it moved"});

    std::vector<std::vector<json::Value>> rows;
    rows.reserve(checks.size());
    std::vector<std::string> rejected;
    std::vector<std::string> overridden;
    std::size_t hard_failures = 0;
    std::size_t policy_failures = 0;
    for (const CheckRecord& check : checks) {
        std::string verdict;
        if (check.passed) {
            verdict = "pass";
        } else if (check.overridable) {
            ++policy_failures;
            verdict = allow_overrides ? "overridden" : "rejected";
            if (allow_overrides) {
                overridden.push_back(check.name);
            } else {
                rejected.push_back(check.name);
            }
        } else {
            ++hard_failures;
            verdict = "rejected";
            rejected.push_back(check.name);
        }
        rows.push_back({json::Value(check.name), json::Value(check.category),
                        json::Value(check.proposed), json::Value(check.limit),
                        json::Value(check.unit), json::Value(verdict),
                        json::Value(check.overridable), json::Value(check.reason)});
    }

    const bool accepted =
        (hard_failures == 0) && (policy_failures == 0 || allow_overrides);

    json::Value out = json::Value::object();
    out.set("decision_record",
            json::from_table({"check", "category", "proposed", "limit", "unit", "verdict",
                              "overridable", "reason"},
                             rows));
    out.set("q_proposed", json::from_vec6(q_proposed));
    out.set("q_current", json::from_vec6(q_current));
    out.set("T_proposed", json::from_isometry(proposed_pose));
    out.set("accepted", json::Value(accepted));
    out.set("rejected", json::Value(!accepted));
    const auto join = [](const std::vector<std::string>& names) {
        std::string out_text;
        for (std::size_t i = 0; i < names.size(); ++i) {
            if (i != 0) {
                out_text += "; ";
            }
            out_text += names[i];
        }
        return out_text.empty() ? std::string("none") : out_text;
    };
    out.set("rejected_checks", json::Value(join(rejected)));
    out.set("overridden_checks", json::Value(join(overridden)));
    out.set("non_overridable_failures", json::Value(static_cast<double>(hard_failures)));
    out.set("overridable_failures", json::Value(static_cast<double>(policy_failures)));
    out.set("check_count", json::Value(static_cast<double>(checks.size())));
    out.set("proposal_source", json::Value(source));
    out.set("allow_overrides", json::Value(allow_overrides));
    out.set("tool_speed", json::Value(tool_speed));
    out.set("horizontal_reach", json::Value(horizontal_reach));
    out.set("verdict",
            json::Value(accepted ? "Accepted: every hard limit passed" +
                                       std::string(overridden.empty()
                                                       ? " and no cell policy was breached."
                                                       : " and the breached cell policies were "
                                                         "explicitly overridden.")
                                 : "Rejected: " + std::to_string(hard_failures) +
                                       " hard limit(s) and " +
                                       std::to_string(allow_overrides ? 0 : policy_failures) +
                                       " cell policy check(s) failed. A hard limit is never "
                                       "overridable - the row says so, and the code honours it."));
    out.set("note",
            json::Value("This is the audit trail, not a reassurance: it names what was proposed, "
                        "which model proposed it, every limit it was checked against with the "
                        "actual numbers, and what was rejected and why. The two categories are "
                        "separated on purpose - a datasheet travel or speed limit is physics and "
                        "nobody may override it, while a collaborative speed or a table "
                        "clearance is cell policy and a named human may accept the risk. A "
                        "learned model that cannot pass this panel does not get to move the "
                        "robot."));
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// learning:: free functions
// ---------------------------------------------------------------------------

namespace learning {

double binary_entropy(std::size_t positives, std::size_t total) noexcept {
    if (total == 0 || positives == 0 || positives == total) {
        return 0.0;
    }
    const double p = static_cast<double>(positives) / static_cast<double>(total);
    const double q = 1.0 - p;
    return -(p * std::log2(p) + q * std::log2(q));
}

PlanarSubChain gp8_planar_subchain() noexcept {
    PlanarSubChain chain;
    chain.l1 = GP8_DH[1].a;  // 0.345 m upper arm
    chain.l2 = GP8_DH[3].d;  // 0.340 m forearm
    return chain;
}

Eigen::Vector2d planar_forward(const PlanarSubChain& chain, double q1, double q2) noexcept {
    const double reach = chain.l1 * std::cos(q1) + chain.l2 * std::cos(q1 + q2);
    const double height = chain.l1 * std::sin(q1) + chain.l2 * std::sin(q1 + q2);
    return Eigen::Vector2d(reach, height);
}

}  // namespace learning

// ---------------------------------------------------------------------------
// Self-description
// ---------------------------------------------------------------------------

ModuleDescription LearningModelsModule::describe() const {
    ModuleDescription d;
    d.name = "learning_models";
    d.title = "Machine Learning, Neural Networks, Evolution and Agents on the GP8";
    d.course = CourseRef{2953, "M-409-01", "Intelligent Systems 1"};
    d.topics = {
        "Introduction to Neural Networks · ANN · backpropagation",
        "Evolutionary algorithms · Genetic Algorithms and Genetic Programming",
        "ML Models · decision trees and model performance evaluation",
        "Intelligent agents · reflex, model-based and goal-based architectures",
        "Closure Lecture · Best Practices and Responsible AI",
    };
    d.source = "cpp_solver/include/study/learning_models.hpp";
    d.summary =
        "Four learning methods of course 2953 written out rather than imported, each learning "
        "something about this GP8: a backpropagation network fitting the planar L-U inverse "
        "kinematics, a genetic algorithm tuning the L-axis PID gains, an information-gain "
        "decision tree predicting limit violations, three agent architectures compared on one "
        "pick-and-place tray, and the Responsible AI panel that audits any motion they propose.";

    const Eigen::Matrix<double, 6, 1> zero6 = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> proposed_default = Eigen::Matrix<double, 6, 1>::Zero();
    proposed_default(1) = 0.9;
    proposed_default(2) = -0.5;

    {
        OpSpec op;
        op.name = "neural_network";
        op.title = "Feedforward network trained by backpropagation on the inverse kinematics";
        op.formula =
            "a = \\tanh(W_1 x + b_1), \\quad \\hat{y} = W_2 a + b_2, \\quad "
            "E = \\tfrac{1}{2}\\lVert \\hat{y} - y \\rVert^2, \\qquad "
            "\\delta_2 = \\hat{y} - y, \\quad \\delta_1 = (W_2^{T}\\delta_2) \\odot "
            "(1 - a \\odot a), \\quad W \\leftarrow W - \\eta\\,\\delta\\,x^{T}";
        op.explain =
            "The network learns the inverse kinematics of the GP8's planar L-U sub-chain: from a "
            "reach and a height to the two joint angles that reach it, on the elbow-down branch "
            "so the map it has to learn is single-valued. The backpropagation is written out "
            "above and in the code: the output layer is linear with a squared error so its delta "
            "is the residual itself, and the hidden delta is the weighted residual times the "
            "tanh derivative 1 - tanh^2. The honest comparison is against a trivial model that "
            "always predicts the training mean, and both numbers come back - a learning curve "
            "that falls means nothing until the final error beats that baseline.";
        op.params = {
            ParamSpec::integer("hidden", "Hidden units", "", 2, 64, 14),
            ParamSpec::scalar("learning_rate", "Learning rate", "", 1e-4, 1.0, 0.05),
            ParamSpec::integer("epochs", "Training epochs", "", 5, 5000, 400),
            ParamSpec::integer("samples", "Dataset size", "", 32, 4000, 400),
            ParamSpec::integer("seed", "Random seed", "", kSeedMin, kSeedMax, 31),
        };
        op.outputs = {
            OutputSpec::make("loss_curve", "series_set", "Training and test loss per epoch"),
            OutputSpec::make("learned_vs_true", "series_set", "Learned against true joint angles"),
            OutputSpec::make("architecture", "table", "Layer sizes and activations"),
            OutputSpec::make("final_error", "scalar", "Held-out RMS joint error", "rad"),
            OutputSpec::make("baseline_error", "scalar", "RMS of the predict-the-mean baseline",
                             "rad"),
            OutputSpec::make("improvement_factor", "scalar", "baseline / network"),
            OutputSpec::make("worst_test_error", "scalar", "Largest single-joint error", "rad"),
            OutputSpec::make("note", "text", "Both numbers, stated"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "genetic_algorithm";
        op.title = "Genetic algorithm tuning the L-axis PID gains";
        op.formula =
            "f(K) = \\frac{1}{1 + 10\\int e^2 dt + 5\\,M_p + 10^{-5}\\int \\tau^2 dt}, \\qquad "
            "c = \\beta a + (1-\\beta) b, \\qquad g \\leftarrow g + \\sigma\\,\\mathcal{N}(0,1) "
            "\\text{ with probability } p_m";
        op.explain =
            "The genome is the three PID gains, normalised to the unit cube so one mutation sigma "
            "means the same thing for all of them, and the fitness is a step response of the real "
            "L-axis drive: the reflected rotor inertia and the link inertia from "
            "gp8_model.hpp, the viscous friction, and the torque ceiling that clips the "
            "controller. Selection is a tournament, crossover is a blend along the segment "
            "between two parents, and mutation is a clipped Gaussian - three named operators, no "
            "black box. With elitism on, the best individual is copied forward unchanged, so the "
            "best-so-far curve cannot step down; the diversity curve collapsing underneath it is "
            "selection pressure made visible.";
        op.params = {
            ParamSpec::integer("population", "Population size", "", 4, 200, 40),
            ParamSpec::integer("generations", "Generations", "", 2, 200, 30),
            ParamSpec::scalar("mutation_rate", "Mutation probability per gene", "", 0.0, 1.0,
                              0.25),
            ParamSpec::scalar("mutation_sigma", "Mutation sigma", "", 0.0, 1.0, 0.12),
            ParamSpec::scalar("crossover_rate", "Crossover probability", "", 0.0, 1.0, 0.8),
            ParamSpec::boolean("elitism", "Carry the best individual forward", true),
            ParamSpec::integer("tournament", "Tournament size", "", 2, 16, 3),
            ParamSpec::scalar("setpoint", "Step setpoint", "rad", 0.01, 1.5, 0.5),
            ParamSpec::integer("seed", "Random seed", "", kSeedMin, kSeedMax, 41),
        };
        op.outputs = {
            OutputSpec::make("curves", "series_set",
                             "Best fitness, mean fitness and gene diversity per generation"),
            OutputSpec::make("step_response", "series_set", "Step response of the winner"),
            OutputSpec::make("best_genome", "table", "Final gains, normalised and physical"),
            OutputSpec::make("best_fitness", "scalar", "Fitness of the winner"),
            OutputSpec::make("best_fitness_monotone", "bool",
                             "Whether the best-so-far curve never fell"),
            OutputSpec::make("diversity_initial", "scalar", "Gene diversity at generation 1"),
            OutputSpec::make("diversity_final", "scalar", "Gene diversity at the last generation"),
            OutputSpec::make("settling_time", "scalar", "2 % settling time of the winner", "s"),
            OutputSpec::make("note", "text", "What each operator contributes"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "decision_tree";
        op.title = "Information-gain tree: will this motion violate a limit?";
        op.formula =
            "H(S) = -p\\log_2 p - (1-p)\\log_2(1-p), \\qquad "
            "IG(S, t) = H(S) - \\frac{|S_L|}{|S|}H(S_L) - \\frac{|S_R|}{|S|}H(S_R)";
        op.explain =
            "The label is not invented: a sampled configuration, speed scale and payload is a "
            "violation when the move would drive the L or U axis past its datasheet travel "
            "inside the horizon, or when the payload's static moment exceeds the L-axis torque "
            "ceiling. The tree then picks, at every node, the feature and threshold with the "
            "largest information gain in bits, which is the parent's entropy minus the weighted "
            "entropy of the two children. The reason to use a tree here rather than something "
            "stronger is that each rule is readable and can be checked against the datasheet by "
            "hand - and because a false negative is a robot driven into its own limit, the "
            "confusion table matters more than the headline accuracy.";
        op.params = {
            ParamSpec::integer("samples", "Dataset size", "", 40, 5000, 600),
            ParamSpec::integer("max_depth", "Maximum depth", "", 1, 10, 4),
            ParamSpec::integer("min_samples_split", "Minimum samples to split", "", 2, 500, 12),
            ParamSpec::integer("min_samples_leaf", "Minimum samples in a leaf", "", 1, 250, 4),
            ParamSpec::integer("candidate_thresholds", "Candidate thresholds per feature", "", 3,
                               63, 15),
            ParamSpec::scalar("horizon", "Motion horizon", "s", 0.01, 2.0, 0.35),
            ParamSpec::integer("seed", "Random seed", "", kSeedMin, kSeedMax, 53),
        };
        op.outputs = {
            OutputSpec::make("tree", "table", "Every node: rule, gain, entropy and counts"),
            OutputSpec::make("information_gain", "series", "Information gain at each split"),
            OutputSpec::make("train_accuracy", "scalar", "Accuracy on the training split"),
            OutputSpec::make("test_accuracy", "scalar", "Accuracy on the held-out split"),
            OutputSpec::make("majority_baseline_accuracy", "scalar",
                             "Accuracy of always predicting the majority class"),
            OutputSpec::make("confusion", "table", "Held-out confusion counts"),
            OutputSpec::make("root_entropy", "scalar", "Entropy of the whole dataset", "bit"),
            OutputSpec::make("note", "text", "Why interpretability is the point here"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "agent_policy";
        op.title = "Reflex, model-based and goal-based agents on one pick-and-place tray";
        op.formula =
            "\\text{reflex: } a = f(p), \\qquad \\text{model-based: } s' = u(s, p),\\; "
            "a = f(s'), \\qquad \\text{goal-based: } a = \\arg\\max_a U(a \\mid s', G), \\quad "
            "U = \\frac{R}{C(a)}";
        op.explain =
            "Three architectures from the course, on the identical seeded tray, so the "
            "comparison is about the architecture and nothing else. The reflex agent maps the "
            "current percept straight to an action and keeps no state, so every rotated part "
            "costs it a failed grip it cannot learn from. The model-based agent carries internal "
            "state and turns that percept into a reorientation first, which is why it places "
            "every part. The goal-based agent adds an explicit goal and a utility of reward over "
            "cost, so it also chooses the ORDER of the work - the only thing left to win once "
            "failures are gone. The trace shows the percept, the internal state, the action and "
            "the reason on every step.";
        op.params = {
            ParamSpec::enumeration("architecture", "Architecture to report",
                                   {"all", "simple_reflex", "model_based", "goal_based_utility"},
                                   "all"),
            ParamSpec::integer("parts", "Parts in the tray", "", 1, 24, 6),
            ParamSpec::scalar("misorientation_rate", "Fraction of rotated parts", "", 0.0, 1.0,
                              0.4),
            ParamSpec::integer("seed", "Random seed", "", kSeedMin, kSeedMax, 61),
        };
        op.outputs = {
            OutputSpec::make("world", "table", "The tray every agent was given"),
            OutputSpec::make("scores", "table", "Placed, failed grips, time cost and score"),
            OutputSpec::make("action_sequences", "table", "Each agent's actions in order"),
            OutputSpec::make("decision_trace", "table",
                             "Percept, internal state, action and reason per step"),
            OutputSpec::make("best_architecture", "text", "Which architecture scored highest"),
            OutputSpec::make("best_score", "scalar", "That architecture's score"),
            OutputSpec::make("note", "text", "What each architecture adds"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "responsible_ai";
        op.title = "Safety and explainability audit of a proposed motion";
        op.formula =
            "\\text{accept} \\iff \\bigwedge_{i \\in \\text{hard}} c_i \\;\\wedge\\; "
            "\\left(\\bigwedge_{i \\in \\text{policy}} c_i \\;\\vee\\; \\text{override}\\right), "
            "\\qquad |\\dot q_j| = \\frac{|q_j^{*} - q_j|}{T} \\le \\dot q_{j,\\max}";
        op.explain =
            "Any of the models above can propose a motion; this op is what stands between the "
            "proposal and the robot. Every check names the quantity, the number that was "
            "proposed, the limit it was compared against and the verdict, so the record is an "
            "audit trail rather than a reassurance. The two categories are separated because "
            "they are different kinds of claim: a datasheet travel, speed, payload or reach "
            "limit is physics and is NOT overridable by anyone, while a collaborative speed or a "
            "table clearance is cell policy that a named human may accept the risk on. A learned "
            "model that cannot pass this panel does not get to move the machine, and the reason "
            "it failed is written down.";
        op.params = {
            ParamSpec::vec6("q_current", "Current joint configuration", "rad",
                            -widest_joint_range(), widest_joint_range(), zero6),
            ParamSpec::vec6("q_proposed", "Proposed joint configuration", "rad",
                            -widest_joint_range(), widest_joint_range(), proposed_default),
            ParamSpec::scalar("duration", "Motion duration", "s", 0.01, 60.0, 1.0),
            ParamSpec::scalar("payload", "Payload at the flange", "kg", 0.0, 40.0, 3.0),
            ParamSpec::enumeration("source", "Which model proposed the motion",
                                   {"neural_network", "genetic_algorithm", "decision_tree",
                                    "agent_policy", "human_operator"},
                                   "neural_network"),
            ParamSpec::boolean("allow_overrides", "A named human accepts the policy risks", false),
            ParamSpec::scalar("collaborative_speed_limit", "Collaborative tool speed limit", "m/s",
                              0.01, 5.0, 0.25),
            ParamSpec::scalar("minimum_height", "Minimum flange height", "m", 0.0, 2.0, 0.20),
        };
        op.outputs = {
            OutputSpec::make("decision_record", "table",
                             "Every check with its number, limit, verdict and reason"),
            OutputSpec::make("accepted", "bool", "Whether the motion may run"),
            OutputSpec::make("rejected_checks", "text", "Names of the checks that failed"),
            OutputSpec::make("overridden_checks", "text", "Policy checks a human waived"),
            OutputSpec::make("non_overridable_failures", "int", "Hard limits that failed"),
            OutputSpec::make("overridable_failures", "int", "Cell policies that failed"),
            OutputSpec::make("T_proposed", "mat4", "Flange pose the proposal would reach"),
            OutputSpec::make("verdict", "text", "The decision, with its arithmetic"),
            OutputSpec::make("note", "text", "Why the two categories are kept apart"),
        };
        d.ops.push_back(std::move(op));
    }
    return d;
}

json::Value LearningModelsModule::invoke(std::string_view op, const json::Value& args) const {
    if (op == "neural_network") {
        return op_neural_network(args);
    }
    if (op == "genetic_algorithm") {
        return op_genetic_algorithm(args);
    }
    if (op == "decision_tree") {
        return op_decision_tree(args);
    }
    if (op == "agent_policy") {
        return op_agent_policy(args);
    }
    if (op == "responsible_ai") {
        return op_responsible_ai(args);
    }
    unknown_op(name(), op);
}

}  // namespace yaskawa::study
