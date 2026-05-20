#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <random>

#include "imgui.h"
#include "imgui-SFML.h"

[[nodiscard]]
sf::Vector2f rotate_vec(const sf::Vector2f &v, float theta)
{
    float c = cos(theta);
    float s = sin(theta);

    return {v.x * c - v.y * s, v.x * s + v.y * c};
}

struct Boid
{
    sf::Vector2f position;
    sf::Vector2f velocity;
    sf::Vector2f acceleration;

    const float max_velocity = 10.0f;
    const float max_steer = 5.0f;

    float perception_radius = 5.0f;

    sf::VertexArray mesh;
    sf::Color color = sf::Color::White;

    Boid(const sf::Vector2f &position = {}, const sf::Vector2f &velocity = {0.0f, 1.0f}) : position(position), velocity(velocity)
    {
        mesh.setPrimitiveType(sf::PrimitiveType::Triangles);
        mesh.resize(3);
        update_mesh();
    }

    void update(float dt)
    {
        update_physics(dt);
        update_mesh();
    }

    void apply_force(const sf::Vector2f &force)
    {
        acceleration += force;
    }

    void update_physics(float dt)
    {
        velocity += acceleration * dt;
        if (velocity.length() > max_velocity)
        {
            velocity = max_velocity * velocity.normalized();
        }

        position += velocity * dt;
        acceleration *= 0.0f;
    }

    void steer(const sf::Vector2f &target_position)
    {
        sf::Vector2f desired_direction = target_position - position;
        if (desired_direction.lengthSquared() < 1e-8)
        {
            return;
        }

        desired_direction = max_velocity * desired_direction.normalized();
        sf::Vector2f steering_force = desired_direction - velocity;

        if (steering_force.lengthSquared() > max_steer * max_steer)
        {
            steering_force = max_steer * steering_force.normalized();
        }

        apply_force(steering_force);
    }

    void set_color(const sf::Color &new_color)
    {
        color = new_color;
        update_mesh();
    }

    void update_mesh()
    {
        float angle = atan2(velocity.y, velocity.x);

        mesh[0].position = rotate_vec({-0.5f, -0.5f}, angle) + position;
        mesh[1].position = rotate_vec({1.0f, 0.0f}, angle) + position;
        mesh[2].position = rotate_vec({-0.5f, 0.5f}, angle) + position;

        mesh[0].color = color;
        mesh[1].color = color;
        mesh[2].color = color;
    }
};

struct WorldConfig
{
    float xmin, xmax, ymin, ymax;
    float vmin, vmax;
    int n;
    int seed;
};

// Returns indices of boids closest to the i-th boid
[[nodiscard]]
std::vector<std::size_t> find_nearest_boids(std::size_t i, const std::vector<Boid> &boids)
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

void alignment(Boid &b, const std::vector<Boid> &boids, const std::vector<std::size_t> &neighbors, float weight)
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

void cohesion(Boid &b, const std::vector<Boid> &boids, const std::vector<std::size_t> &neighbors, float weight)
{
    sf::Vector2f average_position;
    for (const auto j : neighbors)
    {
        average_position += boids[j].position;
    }

    average_position /= static_cast<float>(neighbors.size());

    sf::Vector2f target = average_position * weight + b.position * (1.0f - weight);
    b.steer(target);

    // b.steer(average_position);
}

void separation(Boid &b, const std::vector<Boid> &boids, const std::vector<std::size_t> &neighbors, float weight)
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

void apply_boid_rules(std::vector<Boid> &boids, float w_alignment, float w_cohesion, float w_separation)
{
    std::vector<Boid> new_boids = boids;

    for (std::size_t i = 0; i < boids.size(); ++i)
    {
        auto nearest = find_nearest_boids(i, boids);

        if (nearest.empty())
        {
            continue;
        }

        alignment(new_boids[i], boids, nearest, w_alignment);
        cohesion(new_boids[i], boids, nearest, w_cohesion);
        separation(new_boids[i], boids, nearest, w_separation);
    }

    std::swap(boids, new_boids);
}

std::vector<Boid> create_random_boids(const WorldConfig &config)
{
    std::mt19937 rd(config.seed);

    std::uniform_real_distribution<float> dist_x(config.xmin, config.xmax);
    std::uniform_real_distribution<float> dist_y(config.ymin, config.ymax);
    std::uniform_real_distribution<float> dist_vx(config.vmin, config.vmax);
    std::uniform_real_distribution<float> dist_vy(config.vmin, config.vmax);

    std::vector<Boid> boids(config.n);
    for (std::size_t i = 0; i < config.n; ++i)
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

int main()
{
    WorldConfig config{
        -50.0f, 50.0f, -50.0f, 50.0f,
        -1.0f, 1.0f,
        100,
        42};

    sf::Vector2f world_center = sf::Vector2f(config.xmax + config.xmin, config.ymax + config.ymin) * 0.5f;
    sf::Vector2f world_size = sf::Vector2f(config.xmax - config.xmin, config.ymax - config.ymin);

    sf::RenderWindow window(sf::VideoMode({800, 600}), "My window");
    sf::View view(world_center, world_size);

    window.setView(view);

    auto boids = create_random_boids(config);

    sf::Clock clock;
    float acc = 0.0f;
    float dt = 1.0f / 60.0f;

    sf::Clock imgui_clock;

    if (!ImGui::SFML::Init(window))
    {
        std::cerr << "Could not initialize ImGui\n";
        return 1;
    }

    while (window.isOpen())
    {
        acc += clock.restart().asSeconds();

        while (const std::optional event = window.pollEvent())
        {
            ImGui::SFML::ProcessEvent(window, *event);

            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }
        }

        ImGui::SFML::Update(window, imgui_clock.restart());

        ImGui::Begin("Hello World");
        ImGui::Button("Test button");
        ImGui::End();

        while (acc >= dt)
        {
            apply_boid_rules(boids, 0.1f, 0.05f, 10.0f);

            for (auto &b : boids)
            {
                b.update(dt);

                if (b.position.x > config.xmax)
                {
                    b.position.x = config.xmin;
                }

                if (b.position.x < config.xmin)
                {
                    b.position.x = config.xmax;
                }

                if (b.position.y > config.ymax)
                {
                    b.position.y = config.ymin;
                }

                if (b.position.y < config.ymin)
                {
                    b.position.y = config.ymax;
                }
            }

            acc -= dt;
        }

        window.clear(sf::Color::Black);

        for (auto &b : boids)
        {
            window.draw(b.mesh);
        }

        ImGui::SFML::Render(window);

        window.display();
    }

    ImGui::SFML::Shutdown();
}