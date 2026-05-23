#pragma once

#include "boid.hpp"
#include <optional>

// About the world: a rectangle where boids spawn
struct WorldConfig
{
    float xmin = {};
    float xmax = {};
    float ymin = {};
    float ymax = {};
};

// About the boids: speed, steering, perception radius etc.
struct BoidConfig
{
    float vmin = {};
    float vmax = {};
    float steermin = {};
    float steermax = {};
    float alignment_radius_min = {};
    float alignment_radius_max = {};
    float cohesion_radius_min = {};
    float cohesion_radius_max = {};
    float separation_radius_min = {};
    float separation_radius_max = {};
};

// About the simulation: seed, boid count, rule weights
struct SimConfig
{
    int seed = {};
    std::size_t count = {};
    float dt = {};
    float w_alignment = {};
    float w_cohesion = {};
    float w_separation = {};
};

class Simulation
{
public:
    Simulation(const WorldConfig& world_config, const BoidConfig& boid_config,
               const SimConfig& sim_config);
    ~Simulation() = default;

    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;
    Simulation(Simulation&&) = default;
    Simulation& operator=(Simulation&&) = default;

    void step(bool enable_debug = false);

    [[nodiscard]]
    const std::vector<Boid>& get_boids() const noexcept;

    [[nodiscard]]
    const std::vector<BoidDebugData>& get_debug_data() const noexcept;

private:
    [[nodiscard]]
    std::vector<Boid> create_random_boids() const;

    void apply_boid_rules(bool enable_debug);
    void handle_boundaries();

    // Returns indices of boids closest to the i-th boid
    [[nodiscard]]
    static std::vector<std::size_t>
    find_nearest_boids(std::size_t i, const std::vector<Boid>& snapshot, float perception_radius);

    static void
    alignment(Boid& b, const std::vector<Boid>& snapshot, const std::vector<std::size_t>& neighbors,
              float weight,
              std::optional<std::reference_wrapper<BoidDebugData>> debug = std::nullopt);
    static void cohesion(Boid& b, const std::vector<Boid>& snapshot,
                         const std::vector<std::size_t>& neighbors, float weight,
                         std::optional<std::reference_wrapper<BoidDebugData>> debug = std::nullopt);
    static void
    separation(Boid& b, const std::vector<Boid>& snapshot,
               const std::vector<std::size_t>& neighbors, float weight,
               std::optional<std::reference_wrapper<BoidDebugData>> debug = std::nullopt);

private:
    WorldConfig world_config;
    BoidConfig boid_config;
    SimConfig sim_config;

    std::vector<Boid> boids;
    std::vector<BoidDebugData> debug_data;
};
