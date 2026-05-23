#include "boid.hpp"
#include "utils.hpp"

constexpr float vector_length_threshold = 1e-8f;

[[nodiscard]]
sf::Color create_highlight_color(const sf::Color& color) noexcept
{
    const std::uint8_t r = static_cast<std::uint8_t>(0.25f * static_cast<float>(color.r)) + 64;
    const std::uint8_t g = static_cast<std::uint8_t>(0.25f * static_cast<float>(color.g)) + 64;
    const std::uint8_t b = static_cast<std::uint8_t>(0.25f * static_cast<float>(color.b)) + 64;

    return {r, g, b};
}

sf::CircleShape create_debug_circle(float radius, const sf::Vector2f& position,
                                    const sf::Color& color) noexcept
{
    sf::CircleShape circle(radius);
    circle.setOrigin({radius, radius});
    circle.setPosition(position);
    circle.setFillColor(sf::Color::Transparent);
    circle.setOutlineColor(color);
    circle.setOutlineThickness(0.1f);

    return circle;
}

Boid::Boid(const sf::Vector2f& position, const sf::Vector2f& velocity)
    : position(position), velocity(velocity)
{
}

void Boid::update(float dt)
{
    velocity += acceleration * dt;
    velocity = velocity.lengthSquared() >= vector_length_threshold
                   ? max_velocity * velocity.normalized()
                   : sf::Vector2f{};
    position += velocity * dt;
    acceleration *= 0.0f;
}

void Boid::steer(const sf::Vector2f& target_position)
{
    sf::Vector2f desired_direction = target_position - position;
    if (desired_direction.lengthSquared() < vector_length_threshold)
    {
        return;
    }

    desired_direction = max_velocity * desired_direction.normalized();
    sf::Vector2f steering_force = desired_direction - velocity;
    if (steering_force.lengthSquared() < vector_length_threshold)
    {
        return;
    }

    steering_force = max_steer * steering_force.normalized();
    apply_force(steering_force);
}

void Boid::apply_force(const sf::Vector2f& force)
{
    acceleration += force;
}

sf::VertexArray Boid::mesh() const noexcept
{
    sf::VertexArray mesh(sf::PrimitiveType::Triangles, 3);

    const float angle = std::atan2(velocity.y, velocity.x);

    mesh[0].position = rotate_vec({-0.5f, -0.5f}, angle) + position;
    mesh[1].position = rotate_vec({1.0f, 0.0f}, angle) + position;
    mesh[2].position = rotate_vec({-0.5f, 0.5f}, angle) + position;

    mesh[0].color = color;
    mesh[1].color = color;
    mesh[2].color = color;

    return mesh;
}

sf::VertexArray Boid::highlight() const noexcept
{
    sf::VertexArray mesh(sf::PrimitiveType::Triangles, 3);

    const float angle = std::atan2(velocity.y, velocity.x);
    const sf::Color highlight_color = create_highlight_color(color);

    mesh[0].position = rotate_vec({-0.375f, -0.375f}, angle) + position;
    mesh[1].position = rotate_vec({0.75f, 0.0f}, angle) + position;
    mesh[2].position = rotate_vec({-0.375f, 0.375f}, angle) + position;

    mesh[0].color = highlight_color;
    mesh[1].color = highlight_color;
    mesh[2].color = highlight_color;

    return mesh;
}

sf::CircleShape Boid::debug_alignment_radius() const noexcept
{
    return create_debug_circle(alignment_radius, position, sf::Color::Red);
}

sf::CircleShape Boid::debug_cohesion_radius() const noexcept
{
    return create_debug_circle(cohesion_radius, position, sf::Color::Green);
}

sf::CircleShape Boid::debug_separation_radius() const noexcept
{
    return create_debug_circle(separation_radius, position, sf::Color::Blue);
}
