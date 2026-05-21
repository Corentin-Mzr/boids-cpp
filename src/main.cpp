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
    WorldConfig world_config(-50.0f, 50.0f, -50.0f, 50.0f);
    BoidConfig boid_config(4.0f, 4.0f, 12.0f, 12.0f, 16.0f);
    SimConfig sim_config(42, 100, 1.0f / 60.0f);

    sim_config.w_alignment = 0.5f;
    sim_config.w_cohesion = 0.0f;
    sim_config.w_separation = 1.0f;

    Simulation simulation(world_config, boid_config, sim_config);

    auto world_center =
        sf::Vector2f(world_config.xmax + world_config.xmin, world_config.ymax + world_config.ymin) *
        0.5f;
    sf::Vector2f world_size(world_config.xmax - world_config.xmin,
                            world_config.ymax - world_config.ymin);

    sf::RenderWindow window(sf::VideoMode({800, 600}), "My window");
    sf::View view(world_center, world_size);

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

        window.clear(sf::Color::Black);

        for (const auto& b : simulation.get_boids())
        {
            window.draw(b.mesh());
        }

        ImGui::SFML::Render(window);

        window.display();
    }

    ImGui::SFML::Shutdown();
}