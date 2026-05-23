#include "simulation.hpp"
#include <random>

Simulation::Simulation(const WorldConfig& world_config, const BoidConfig& boid_config,
                       const SimConfig& sim_config)
    : world_config(world_config), boid_config(boid_config), sim_config(sim_config)
{
    boids = create_random_boids();
    debug_data.resize(boids.size());
}

void Simulation::step(bool enable_debug)
{
    debug_data.assign(boids.size(), {});
    apply_boid_rules(enable_debug);

    for (auto& b : boids)
    {
        b.update(sim_config.dt);
    }

    handle_boundaries();
}

const std::vector<Boid>& Simulation::get_boids() const noexcept
{
    return boids;
}

const std::vector<BoidDebugData>& Simulation::get_debug_data() const noexcept
{
    return debug_data;
}

std::vector<Boid> Simulation::create_random_boids() const
{
    std::mt19937 rd(sim_config.seed);

    std::uniform_real_distribution<float> dist_x(world_config.xmin, world_config.xmax);
    std::uniform_real_distribution<float> dist_y(world_config.ymin, world_config.ymax);

    std::uniform_real_distribution<float> dist_steer(boid_config.steermin, boid_config.steermax);
    std::uniform_real_distribution<float> dist_v(boid_config.vmin, boid_config.vmax);

    std::uniform_real_distribution<float> dist_vx(-1.0f, 1.0f);
    std::uniform_real_distribution<float> dist_vy(-1.0f, 1.0f);

    std::uniform_real_distribution<float> dist_radius_align(boid_config.alignment_radius_min,
                                                            boid_config.alignment_radius_max);
    std::uniform_real_distribution<float> dist_radius_cohesion(boid_config.cohesion_radius_min,
                                                               boid_config.cohesion_radius_max);
    std::uniform_real_distribution<float> dist_radius_separation(boid_config.separation_radius_min,
                                                                 boid_config.separation_radius_max);

    std::vector<Boid> random_boids(sim_config.count);
    for (std::size_t i = 0; i < sim_config.count; ++i)
    {
        random_boids[i].position = {dist_x(rd), dist_y(rd)};
        random_boids[i].velocity = {dist_vx(rd), dist_vy(rd)};

        random_boids[i].max_steer = dist_steer(rd);
        random_boids[i].max_velocity = dist_v(rd);

        random_boids[i].alignment_radius = dist_radius_align(rd);
        random_boids[i].cohesion_radius = dist_radius_cohesion(rd);
        random_boids[i].separation_radius = dist_radius_separation(rd);
    }

    return random_boids;
}

void Simulation::handle_boundaries()
{
    for (auto& b : boids)
    {
        if (b.position.x > world_config.xmax)
        {
            b.position.x = world_config.xmin;
        }

        if (b.position.x < world_config.xmin)
        {
            b.position.x = world_config.xmax;
        }

        if (b.position.y > world_config.ymax)
        {
            b.position.y = world_config.ymin;
        }

        if (b.position.y < world_config.ymin)
        {
            b.position.y = world_config.ymax;
        }
    }
}

std::vector<std::size_t> Simulation::find_nearest_boids(std::size_t i,
                                                        const std::vector<Boid>& snapshot,
                                                        float perception_radius)
{
    if (i >= snapshot.size())
    {
        return {};
    }

    std::vector<std::size_t> result;
    const sf::Vector2f position = snapshot[i].position;

    for (std::size_t j = 0; j < snapshot.size(); ++j)
    {
        if (i == j)
        {
            continue;
        }

        const float dist_sq = (position - snapshot[j].position).lengthSquared();

        if (dist_sq <= perception_radius * perception_radius)
        {
            result.push_back(j);
        }
    }

    return result;
}

void Simulation::alignment(Boid& b, const std::vector<Boid>& snapshot,
                           const std::vector<std::size_t>& neighbors, float weight,
                           std::optional<std::reference_wrapper<BoidDebugData>> debug)
{
    sf::Vector2f average_velocity;
    for (const auto j : neighbors)
    {
        average_velocity += snapshot[j].velocity;
    }

    average_velocity /= static_cast<float>(neighbors.size());
    average_velocity *= weight;

    const sf::Vector2f steering_direction = b.position + average_velocity;

    if (debug.has_value())
    {
        debug->get().alignment_direction = steering_direction;
        debug->get().alignment_neighbor_count = neighbors.size();
    }

    b.steer(steering_direction);
}

void Simulation::cohesion(Boid& b, const std::vector<Boid>& snapshot,
                          const std::vector<std::size_t>& neighbors, float weight,
                          std::optional<std::reference_wrapper<BoidDebugData>> debug)
{
    sf::Vector2f average_position;
    for (const auto j : neighbors)
    {
        average_position += snapshot[j].position;
    }

    average_position /= static_cast<float>(neighbors.size());

    const sf::Vector2f target = average_position * weight + b.position * (1.0f - weight);

    if (debug.has_value())
    {
        debug->get().cohesion_position = target;
        debug->get().cohesion_neighbor_count = neighbors.size();
    }

    b.steer(target);
}

void Simulation::separation(Boid& b, const std::vector<Boid>& snapshot,
                            const std::vector<std::size_t>& neighbors, float weight,
                            std::optional<std::reference_wrapper<BoidDebugData>> debug)
{
    sf::Vector2f position_diff;
    for (const auto j : neighbors)
    {
        const sf::Vector2f delta = b.position - snapshot[j].position;
        const float dist_sq = delta.lengthSquared();

        if (dist_sq < 1e-8)
        {
            continue;
        }

        const float exp_decr = b.separation_radius / dist_sq;
        position_diff += delta.normalized() * exp_decr;
    }

    position_diff /= static_cast<float>(neighbors.size());
    position_diff *= weight;

    const sf::Vector2f steering_direction = b.position + position_diff;

    if (debug.has_value())
    {
        debug->get().separation_direction = steering_direction;
        debug->get().separation_neighbor_count = neighbors.size();
    }

    b.steer(steering_direction);
}

void Simulation::apply_boid_rules(bool enable_debug)
{
    const std::vector<Boid> snapshot = boids;

    for (std::size_t i = 0; i < boids.size(); ++i)
    {
        const auto nearest_align = find_nearest_boids(i, snapshot, boids[i].alignment_radius);
        const auto nearest_cohesion = find_nearest_boids(i, snapshot, boids[i].cohesion_radius);
        const auto nearest_separation = find_nearest_boids(i, snapshot, boids[i].separation_radius);
        auto debug = enable_debug ? std::optional(std::ref(debug_data[i])) : std::nullopt;

        if (!nearest_align.empty())
        {
            alignment(boids[i], snapshot, nearest_align, sim_config.w_alignment, debug);
        }

        if (!nearest_cohesion.empty())
        {
            cohesion(boids[i], snapshot, nearest_cohesion, sim_config.w_cohesion, debug);
        }

        if (!nearest_separation.empty())
        {
            separation(boids[i], snapshot, nearest_separation, sim_config.w_separation, debug);
        }
    }
}
