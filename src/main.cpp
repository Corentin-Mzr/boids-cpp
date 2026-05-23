#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "imgui-SFML.h"
#include "imgui.h"

#include "boid.hpp"
#include "simulation.hpp"
#include "utils.hpp"

struct DebugState
{
    int hovered_boid_index = -1;
    int selected_boid_index = -1;

    bool show_alignment_radius = true;
    bool show_cohesion_radius = true;
    bool show_separation_radius = true;

    bool show_alignment_direction = true;
    bool show_cohesion_position = true;
    bool show_separation_direction = true;

    bool enable_debug_calculations = true;
    bool show_debug_panel = false;
};

[[nodiscard]]
int find_boid_at_position(const std::vector<Boid>& boids, const sf::Vector2f& world_pos,
                          float hit_radius)
{
    int closest_index = -1;
    float closest_distance = hit_radius * hit_radius;

    for (std::size_t i = 0; i < boids.size(); ++i)
    {
        const float dist_sq = (boids[i].position - world_pos).lengthSquared();
        if (dist_sq < closest_distance)
        {
            closest_index = static_cast<int>(i);
            closest_distance = dist_sq;
        }
    }

    return closest_index;
}

void draw_debug_panel(const Boid& boid, const BoidDebugData& debug_data, DebugState& debug_state)
{
    ImGui::Begin("Debug");
    ImGui::Checkbox("Show Alignment Radius", &debug_state.show_alignment_radius);
    ImGui::Checkbox("Show Cohesion Radius", &debug_state.show_cohesion_radius);
    ImGui::Checkbox("Show Separation Radius", &debug_state.show_separation_radius);

    ImGui::Checkbox("Show Alignment Direction", &debug_state.show_alignment_direction);
    ImGui::Checkbox("Show Cohesion Position", &debug_state.show_cohesion_position);
    ImGui::Checkbox("Show Separation Direction", &debug_state.show_separation_direction);

    ImGui::Separator();
    ImGui::Text("Selected Boid: %d", debug_state.selected_boid_index);

    ImGui::Text("Pos: (%.2f, %.2f)", boid.position.x, boid.position.y);
    ImGui::Text("Vel: (%.2f, %.2f)", boid.velocity.x, boid.velocity.y);
    ImGui::Text("Speed: %.2f", boid.velocity.length());

    if (debug_state.enable_debug_calculations)
    {
        ImGui::Separator();
        ImGui::Text("Alignment direction: (%.2f, %.2f)", debug_data.alignment_direction.x,
                    debug_data.alignment_direction.y);
        ImGui::Text("Alignment Neighbor Count: %d", debug_data.alignment_neighbor_count);

        ImGui::Text("Cohesion position: (%.2f, %.2f)", debug_data.cohesion_position.x,
                    debug_data.cohesion_position.y);
        ImGui::Text("Cohesion Neighbor Count: %d", debug_data.cohesion_neighbor_count);

        ImGui::Text("Separation direction: (%.2f, %.2f)", debug_data.separation_direction.x,
                    debug_data.separation_direction.y);
        ImGui::Text("Separation Neighbor Count: %d", debug_data.separation_neighbor_count);
    }

    ImGui::End();
}

void draw_debug_tools(const Boid& boid, const BoidDebugData& debug_data,
                      const DebugState& debug_state, sf::RenderWindow& window)
{
    if (debug_state.show_alignment_radius)
    {
        window.draw(boid.debug_alignment_radius());
    }

    if (debug_state.show_cohesion_radius)
    {
        window.draw(boid.debug_cohesion_radius());
    }

    if (debug_state.show_separation_radius)
    {
        window.draw(boid.debug_separation_radius());
    }

    if (debug_state.show_alignment_direction && debug_data.alignment_neighbor_count > 0)
    {
        sf::CircleShape circle(0.5f);
        circle.setOrigin({0.5f, 0.5f});
        circle.setPosition(debug_data.alignment_direction);
        circle.setFillColor(sf::Color::Red);
        window.draw(circle);
    }

    if (debug_state.show_cohesion_position && debug_data.cohesion_neighbor_count > 0)
    {
        sf::CircleShape circle(0.5f);
        circle.setOrigin({0.5f, 0.5f});
        circle.setPosition(debug_data.cohesion_position);
        circle.setFillColor(sf::Color::Green);
        window.draw(circle);
    }

    if (debug_state.show_separation_direction && debug_data.separation_neighbor_count > 0)
    {
        sf::CircleShape circle(0.5f);
        circle.setOrigin({0.5f, 0.5f});
        circle.setPosition(debug_data.separation_direction);
        circle.setFillColor(sf::Color::Blue);
        window.draw(circle);
    }
}

int main()
{
    const WorldConfig world_config{.xmin = -25.0f, .xmax = 25.0f, .ymin = -25.0f, .ymax = 25.0f};

    const BoidConfig boid_config{.vmin = 2.0f,
                                 .vmax = 2.0f,
                                 .steermin = 6.0f,
                                 .steermax = 6.0f,
                                 .alignment_radius_min = 5.0f,
                                 .alignment_radius_max = 5.0f,
                                 .cohesion_radius_min = 5.0f,
                                 .cohesion_radius_max = 5.0f,
                                 .separation_radius_min = 5.0f,
                                 .separation_radius_max = 5.0f};

    const SimConfig sim_config{.seed = 42,
                               .count = 100,
                               .dt = 1.0f / 60.0f,
                               .w_alignment = 1.0f,
                               .w_cohesion = 1.0f,
                               .w_separation = 1.0f};

    Simulation simulation(world_config, boid_config, sim_config);

    DebugState debug_state;

    const auto world_center =
        sf::Vector2f(world_config.xmax + world_config.xmin, world_config.ymax + world_config.ymin) *
        0.5f;
    const sf::Vector2f world_size(world_config.xmax - world_config.xmin,
                                  world_config.ymax - world_config.ymin);

    sf::RenderWindow window(sf::VideoMode({1280, 720}), "My window");
    const sf::View view(world_center, world_size);

    window.setView(view);

    sf::Clock imgui_clock;
    sf::Clock clock;
    float acc = 0.0f;

    if (!ImGui::SFML::Init(window))
    {
        std::cerr << "Could not initialize ImGui\n";
        return 1;
    }

    while (window.isOpen())
    {
        acc += clock.restart().asSeconds();

        // Events
        while (const std::optional event = window.pollEvent())
        {
            ImGui::SFML::ProcessEvent(window, *event);

            const ImGuiIO& io = ImGui::GetIO();

            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }

            if (const auto* mouse_move = event->getIf<sf::Event::MouseMoved>())
            {
                if (io.WantCaptureMouse)
                {
                    continue;
                }

                const sf::Vector2f world_pos = window.mapPixelToCoords(mouse_move->position);
                debug_state.hovered_boid_index =
                    find_boid_at_position(simulation.get_boids(), world_pos, 2.0f);
            }

            if (const auto* mouse_click = event->getIf<sf::Event::MouseButtonPressed>())
            {
                if (io.WantCaptureMouse)
                {
                    continue;
                }

                if (mouse_click->button == sf::Mouse::Button::Left)
                {
                    debug_state.selected_boid_index = debug_state.hovered_boid_index;
                    debug_state.show_debug_panel = (debug_state.selected_boid_index != -1);
                }
            }
        }

        // Simulation
        ImGui::SFML::Update(window, imgui_clock.restart());

        while (acc >= sim_config.dt)
        {
            simulation.step(true);
            acc -= sim_config.dt;
        }

        // Rendering
        window.clear({127, 127, 127});

        for (std::size_t i = 0; i < simulation.get_boids().size(); ++i)
        {
            const auto& boid = simulation.get_boids()[i];
            const auto& debug_data = simulation.get_debug_data()[debug_state.selected_boid_index];
            window.draw(boid.mesh());

            if (debug_state.hovered_boid_index == static_cast<int>(i))
            {
                const auto highlight = boid.highlight();
                window.draw(highlight);
            }

            if (debug_state.selected_boid_index == static_cast<int>(i))
            {
                draw_debug_tools(boid, debug_data, debug_state, window);
            }
        }

        if (debug_state.show_debug_panel && debug_state.selected_boid_index >= 0)
        {
            const auto& boid = simulation.get_boids()[debug_state.selected_boid_index];
            const auto& debug_data = simulation.get_debug_data()[debug_state.selected_boid_index];
            draw_debug_panel(boid, debug_data, debug_state);
        }

        ImGui::SFML::Render(window);

        window.display();
    }

    ImGui::SFML::Shutdown();
}
