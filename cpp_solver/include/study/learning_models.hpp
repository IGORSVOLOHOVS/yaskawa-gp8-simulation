#ifndef YASKAWA_STUDY_LEARNING_MODELS_HPP
#define YASKAWA_STUDY_LEARNING_MODELS_HPP

// Course 2953 "Intelligent Systems 1": agents, machine learning, neural
// networks, evolutionary algorithms and the Responsible AI closure block.
//
// Every model here is written out rather than called: the backpropagation is
// four loops, the genetic algorithm's selection, crossover and mutation are
// each a named function, and the decision tree splits on an information gain
// this file computes. There is no external learning library and there will not
// be one - the point of the module is that a student can read the arithmetic.
//
// Every model learns something about THIS robot: the inverse kinematics of the
// GP8's planar L-U sub-chain, the PID gains of its L-axis drive, whether a
// commanded motion violates its limits, and a pick-and-place task in its cell.
// All of it comes from study/gp8_model.hpp.
//
// Determinism: every stochastic op takes an integer `seed` and draws from
// sensing::SeededRng, so a training curve is reproducible to the bit.

#include "study/sensing_models.hpp"
#include "study/study_module.hpp"

#include <Eigen/Dense>

#include <cstddef>
#include <string_view>
#include <vector>

namespace yaskawa::study {

namespace learning {

// Shannon entropy in bits of a binary label set, 0 when pure.
[[nodiscard]] double binary_entropy(std::size_t positives, std::size_t total) noexcept;

// The planar two-link sub-chain of the GP8: the L and U axes moving in the
// vertical plane, with the link lengths taken from the DH table.
struct PlanarSubChain {
    double l1 = 0.0;  // GP8_DH[1].a, the upper-arm length
    double l2 = 0.0;  // GP8_DH[3].d, the forearm length
};

[[nodiscard]] PlanarSubChain gp8_planar_subchain() noexcept;

// (reach, height) of the planar sub-chain's tip for the two joint angles.
[[nodiscard]] Eigen::Vector2d planar_forward(const PlanarSubChain& chain, double q1,
                                             double q2) noexcept;

}  // namespace learning

class LearningModelsModule final : public StudyModule {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "learning_models"; }
    [[nodiscard]] ModuleDescription describe() const override;
    [[nodiscard]] json::Value invoke(std::string_view op, const json::Value& args) const override;
};

}  // namespace yaskawa::study

#endif  // YASKAWA_STUDY_LEARNING_MODELS_HPP
