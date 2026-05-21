#include "utils.hpp"
#include <cmath>

sf::Vector2f rotate_vec(const sf::Vector2f& v, float theta)
{
    float c = std::cos(theta);
    float s = std::sin(theta);

    return {v.x * c - v.y * s, v.x * s + v.y * c};
}
