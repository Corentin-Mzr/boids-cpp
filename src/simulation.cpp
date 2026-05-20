#include "simulation.hpp"
#include <random>

WorldConfig::WorldConfig(float xmin, float xmax, float ymin, float ymax)
    : xmin(xmin), xmax(xmax), ymin(ymin), ymax(ymax)
{
}

BoidConfig::BoidConfig(float vmin, float vmax, float steer, float radius)
    : vmin(vmin), vmax(vmax), steermin(steer), steermax(steer),
      alignment_radius_min(radius), alignment_radius_max(radius),
      cohesion_radius_min(radius), cohesion_radius_max(radius),
      separation_radius_min(radius), separation_radius_max(radius)
{
}

SimConfig::SimConfig(int seed, std::size_t n, float dt)
    : seed(seed), count(n), dt(dt), w_alignment(1.0f), w_cohesion(1.0f), w_separation(1.0f)
{
}

Simulation::Simulation(const WorldConfig &world_config, const BoidConfig &boid_config, const SimConfig &sim_config)
    : world_config(world_config), boid_config(boid_config), sim_config(sim_config)
{
    boids = create_random_boids();
}

void Simulation::step()
{
    apply_boid_rules();

    for (auto &b : boids)
    {
        b.update(sim_config.dt);
    }

    handle_boundaries();
}

const std::vector<Boid> &Simulation::get_boids() const noexcept
{
    return boids;
}

std::vector<Boid> Simulation::create_random_boids()
{
    std::mt19937 rd(sim_config.seed);

    std::uniform_real_distribution<float> dist_x(world_config.xmin, world_config.xmax);
    std::uniform_real_distribution<float> dist_y(world_config.ymin, world_config.ymax);

    std::uniform_real_distribution<float> dist_vx(boid_config.vmin, boid_config.vmax);
    std::uniform_real_distribution<float> dist_vy(boid_config.vmin, boid_config.vmax);

    std::vector<Boid> boids(sim_config.count);
    for (std::size_t i = 0; i < sim_config.count; ++i)
    {
        float x = dist_x(rd);
        float y = dist_y(rd);
        float vx = dist_vx(rd);
        float vy = dist_vy(rd);

        boids[i].position = {x, y};
        boids[i].velocity = {vx, vy};
    }

    return boids;
}

void Simulation::handle_boundaries()
{
    for (auto &b : boids)
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

std::vector<std::size_t> Simulation::find_nearest_boids(std::size_t i, const std::vector<Boid> &boids)
{
    if (i >= boids.size())
    {
        return {};
    }

    std::vector<std::size_t> result;
    sf::Vector2f position = boids[i].position;
    float perception_radius = boids[i].perception_radius;

    for (std::size_t j = 0; j < boids.size(); ++j)
    {
        if (i == j)
        {
            continue;
        }

        float dist_sq = (position - boids[j].position).lengthSquared();

        if (dist_sq <= perception_radius * perception_radius)
        {
            result.push_back(j);
        }
    }

    return result;
}

void Simulation::alignment(Boid &b, const std::vector<Boid> &boids, const std::vector<std::size_t> &neighbors, float weight)
{
    sf::Vector2f average_velocity;
    for (const auto j : neighbors)
    {
        average_velocity += boids[j].velocity;
    }

    average_velocity /= static_cast<float>(neighbors.size());
    average_velocity *= weight;

    b.steer(b.position + average_velocity);
}

void Simulation::cohesion(Boid &b, const std::vector<Boid> &boids, const std::vector<std::size_t> &neighbors, float weight)
{
    sf::Vector2f average_position;
    for (const auto j : neighbors)
    {
        average_position += boids[j].position;
    }

    average_position /= static_cast<float>(neighbors.size());

    sf::Vector2f target = average_position * weight + b.position * (1.0f - weight);
    b.steer(target);
}

void Simulation::separation(Boid &b, const std::vector<Boid> &boids, const std::vector<std::size_t> &neighbors, float weight)
{
    sf::Vector2f position_diff;
    for (const auto j : neighbors)
    {
        sf::Vector2f delta = b.position - boids[j].position;
        float dist_sq = delta.lengthSquared();

        if (dist_sq < 1e-8)
        {
            continue;
        }

        position_diff += delta / dist_sq;
    }

    position_diff /= static_cast<float>(neighbors.size());
    position_diff *= weight;

    b.steer(b.position + position_diff);
}

void Simulation::apply_boid_rules()
{
    std::vector<Boid> new_boids = boids;

    for (std::size_t i = 0; i < boids.size(); ++i)
    {
        auto nearest = find_nearest_boids(i, boids);

        if (nearest.empty())
        {
            continue;
        }

        alignment(new_boids[i], boids, nearest, sim_config.w_alignment);
        cohesion(new_boids[i], boids, nearest, sim_config.w_cohesion);
        separation(new_boids[i], boids, nearest, sim_config.w_separation);
    }

    std::swap(boids, new_boids);
}
