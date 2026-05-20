#pragma once

#include "boid.hpp"

// About the world: a rectangle where boids spawn
struct WorldConfig
{
    float xmin = {}, xmax = {};
    float ymin = {}, ymax = {};

    WorldConfig(float xmin = -10.0, float xmax = 10.0f, float ymin = -10.0f, float ymax = 10.0f);
};

// About the boids: speed, steering, perception radius etc.
struct BoidConfig
{
    float vmin = {}, vmax = {};
    float steermin = {}, steermax = {};
    float alignment_radius_min = {}, alignment_radius_max = {};
    float cohesion_radius_min = {}, cohesion_radius_max = {};
    float separation_radius_min = {}, separation_radius_max = {};

    BoidConfig(float vmin, float vmax, float steer, float radius);
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

    SimConfig(int seed, std::size_t n, float dt);
};

class Simulation
{
public:
    Simulation(const WorldConfig &world_config, const BoidConfig &boid_config, const SimConfig &sim_config);
    ~Simulation() = default;

    void step();

    [[nodiscard]]
    const std::vector<Boid> &get_boids() const noexcept;

private:
    [[nodiscard]]
    std::vector<Boid> create_random_boids();

    void apply_boid_rules();
    void handle_boundaries();

    // Returns indices of boids closest to the i-th boid
    [[nodiscard]]
    std::vector<std::size_t> find_nearest_boids(std::size_t i, const std::vector<Boid> &boids);

    void alignment(Boid &b, const std::vector<Boid> &boids, const std::vector<std::size_t> &neighbors, float weight);
    void cohesion(Boid &b, const std::vector<Boid> &boids, const std::vector<std::size_t> &neighbors, float weight);
    void separation(Boid &b, const std::vector<Boid> &boids, const std::vector<std::size_t> &neighbors, float weight);

private:
    WorldConfig world_config;
    BoidConfig boid_config;
    SimConfig sim_config;

    std::vector<Boid> boids;
};
