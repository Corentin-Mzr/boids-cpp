#include "boid.hpp"
#include "utils.hpp"

Boid::Boid(const sf::Vector2f& position, const sf::Vector2f& velocity)
    : position(position), velocity(velocity)
{
}

void Boid::update(float dt)
{
    velocity += acceleration * dt;
    // if (velocity.length() > max_velocity)
    {
        velocity = max_velocity * velocity.normalized();
    }

    position += velocity * dt;
    acceleration *= 0.0f;
}

void Boid::steer(const sf::Vector2f& target_position)
{
    sf::Vector2f desired_direction = target_position - position;
    if (desired_direction.lengthSquared() < 1e-8)
    {
        return;
    }

    desired_direction = max_velocity * desired_direction.normalized();
    sf::Vector2f steering_force = desired_direction - velocity;

    // if (steering_force.lengthSquared() > max_steer * max_steer)
    {
        steering_force = max_steer * steering_force.normalized();
    }

    apply_force(steering_force);
}

void Boid::apply_force(const sf::Vector2f& force)
{
    acceleration += force;
}

sf::VertexArray Boid::mesh() const noexcept
{
    sf::VertexArray mesh(sf::PrimitiveType::Triangles, 3);

    float angle = std::atan2(velocity.y, velocity.x);

    mesh[0].position = rotate_vec({-0.5f, -0.5f}, angle) + position;
    mesh[1].position = rotate_vec({1.0f, 0.0f}, angle) + position;
    mesh[2].position = rotate_vec({-0.5f, 0.5f}, angle) + position;

    mesh[0].color = color;
    mesh[1].color = color;
    mesh[2].color = color;

    return mesh;
}
