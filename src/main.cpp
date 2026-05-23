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

int main()
{
    const WorldConfig world_config{.xmin = -25.0f, .xmax = 25.0f, .ymin = -25.0f, .ymax = 25.0f};

    const BoidConfig boid_config{.vmin = 2.0f,
                                 .vmax = 2.0f,
                                 .steermin = 6.0f,
                                 .steermax = 6.0f,
                                 .alignment_radius_min = 3.0f,
                                 .alignment_radius_max = 3.0f,
                                 .cohesion_radius_min = 5.0f,
                                 .cohesion_radius_max = 5.0f,
                                 .separation_radius_min = 7.0f,
                                 .separation_radius_max = 7.0f};

    const SimConfig sim_config{.seed = 42,
                               .count = 5,
                               .dt = 1.0f / 60.0f,
                               .w_alignment = 1.0f,
                               .w_cohesion = 1.0f,
                               .w_separation = 1.0f};

    Simulation simulation(world_config, boid_config, sim_config);

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

    int i = 0;

    if (!ImGui::SFML::Init(window))
    {
        std::cerr << "Could not initialize ImGui\n";
        return 1;
    }

    while (window.isOpen())
    {
        i = 0;
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

        while (acc >= sim_config.dt)
        {
            simulation.step();
            acc -= sim_config.dt;
        }

        window.clear({127, 127, 127});

        for (const auto& b : simulation.get_boids())
        {
            window.draw(b.mesh());

            if (i == 0)
            {
                window.draw(b.debug_alignment_radius());
                window.draw(b.debug_cohesion_radius());
                window.draw(b.debug_separation_radius());
            }

            i += 1;
        }

        ImGui::SFML::Render(window);

        window.display();
    }

    ImGui::SFML::Shutdown();
}
