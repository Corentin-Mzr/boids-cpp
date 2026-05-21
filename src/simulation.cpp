#include "simulation.hpp"
#include <random>

WorldConfig::WorldConfig(float xmin, float xmax, float ymin, float ymax)
    : xmin(xmin), xmax(xmax), ymin(ymin), ymax(ymax)
{
}

BoidConfig::BoidConfig(float vmin, float vmax, float steermin, float steermax, float radius)
    : vmin(vmin), vmax(vmax), steermin(steermin), steermax(steermax),
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

    std::uniform_real_distribution<float> dist_steer(boid_config.steermin, boid_config.steermax);
    std::uniform_real_distribution<float> dist_v(boid_config.vmin, boid_config.vmax);

    std::uniform_real_distribution<float> dist_vx(-1.0f, 1.0f);
    std::uniform_real_distribution<float> dist_vy(-1.0f, 1.0f);

    std::uniform_real_distribution<float> dist_radius_align(boid_config.alignment_radius_min, boid_config.alignment_radius_max);
    std::uniform_real_distribution<float> dist_radius_cohesion(boid_config.cohesion_radius_min, boid_config.cohesion_radius_max);
    std::uniform_real_distribution<float> dist_radius_separation(boid_config.separation_radius_min, boid_config.separation_radius_max);

    std::vector<Boid> boids(sim_config.count);
    for (std::size_t i = 0; i < sim_config.count; ++i)
    {
        boids[i].position = {dist_x(rd), dist_y(rd)};
        boids[i].velocity = {dist_vx(rd), dist_vy(rd)};

        boids[i].max_steer = dist_steer(rd);
        boids[i].max_velocity = dist_v(rd);

        boids[i].alignment_radius = dist_radius_align(rd);
        boids[i].cohesion_radius = dist_radius_cohesion(rd);
        boids[i].separation_radius = dist_radius_separation(rd);
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

std::vector<std::size_t> Simulation::find_nearest_boids(std::size_t i, const std::vector<Boid> &snapshot, float perception_radius)
{
    if (i >= snapshot.size())
    {
        return {};
    }

    std::vector<std::size_t> result;
    sf::Vector2f position = snapshot[i].position;

    for (std::size_t j = 0; j < snapshot.size(); ++j)
    {
        if (i == j)
        {
            continue;
        }

        float dist_sq = (position - snapshot[j].position).lengthSquared();

        if (dist_sq <= perception_radius * perception_radius)
        {
            result.push_back(j);
        }
    }

    return result;
}

void Simulation::alignment(Boid &b, const std::vector<Boid> &snapshot, const std::vector<std::size_t> &neighbors, float weight)
{
    sf::Vector2f average_velocity;
    for (const auto j : neighbors)
    {
        average_velocity += snapshot[j].velocity;
    }

    average_velocity /= static_cast<float>(neighbors.size());
    average_velocity *= weight;

    b.steer(b.position + average_velocity);
}

void Simulation::cohesion(Boid &b, const std::vector<Boid> &snapshot, const std::vector<std::size_t> &neighbors, float weight)
{
    sf::Vector2f average_position;
    for (const auto j : neighbors)
    {
        average_position += snapshot[j].position;
    }

    average_position /= static_cast<float>(neighbors.size());

    sf::Vector2f target = average_position * weight + b.position * (1.0f - weight);
    b.steer(target);
}

void Simulation::separation(Boid &b, const std::vector<Boid> &snapshot, const std::vector<std::size_t> &neighbors, float weight)
{
    sf::Vector2f position_diff;
    for (const auto j : neighbors)
    {
        sf::Vector2f delta = b.position - snapshot[j].position;
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
    const std::vector<Boid> snapshot = boids;

    for (std::size_t i = 0; i < boids.size(); ++i)
    {
        auto nearest_align = find_nearest_boids(i, snapshot, boids[i].alignment_radius);
        auto nearest_cohesion = find_nearest_boids(i, snapshot, boids[i].cohesion_radius);
        auto nearest_separation = find_nearest_boids(i, snapshot, boids[i].separation_radius);

        if (!nearest_align.empty())
        {
            alignment(boids[i], snapshot, nearest_align, sim_config.w_alignment);
        }

        if (!nearest_cohesion.empty())
        {
            cohesion(boids[i], snapshot, nearest_cohesion, sim_config.w_cohesion);
        }

        if (!nearest_separation.empty())
        {
            separation(boids[i], snapshot, nearest_separation, sim_config.w_separation);
        }
    }
}
