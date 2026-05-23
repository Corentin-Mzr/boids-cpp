#pragma once

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/VertexArray.hpp>

struct Boid
{
public:
    explicit Boid(const sf::Vector2f& position = {}, const sf::Vector2f& velocity = {0.0f, 1.0f});

    void update(float dt);
    void steer(const sf::Vector2f& target_position);
    void apply_force(const sf::Vector2f& force);

    [[nodiscard]]
    sf::VertexArray mesh() const noexcept;

    [[nodiscard]]
    sf::VertexArray highlight() const noexcept;

    [[nodiscard]]
    sf::CircleShape debug_alignment_radius() const noexcept;

    [[nodiscard]]
    sf::CircleShape debug_cohesion_radius() const noexcept;

    [[nodiscard]]
    sf::CircleShape debug_separation_radius() const noexcept;

    // Physics
    sf::Vector2f position;
    sf::Vector2f velocity;
    sf::Vector2f acceleration;

    float max_velocity = 5.0f;
    float max_steer = 10.0f;

    float alignment_radius = 5.0f;
    float cohesion_radius = 5.0f;
    float separation_radius = 5.0f;

    // Rendering
    sf::Color color = sf::Color::White;
};
