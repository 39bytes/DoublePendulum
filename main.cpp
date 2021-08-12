#include <SFML/Graphics.hpp>
#include "pendulum.h"

void drawPendulum(Pendulum &pendulum, sf::RenderWindow& window)
{
    window.draw(pendulum.line1, 2, sf::Lines);
    window.draw(pendulum.line2, 2, sf::Lines);
    window.draw(pendulum.bob1);
    window.draw(pendulum.bob2);
    window.draw(&pendulum.trail[0], pendulum.trail.size(), sf::LineStrip);
}

int main()
{
    sf::RenderWindow window(sf::VideoMode(400, 400), "Pendulum simulation");
    window.setFramerateLimit(60);
    Pendulum pendulums[1] = {
        Pendulum(sf::Vector2f(200.0f, 100.0f), 1.5, 1.0, 2.0, 2.0, 1.3, 1.0,
        sf::Color::Blue, sf::Color::White, sf::Color::Cyan),
    };

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
        // Render each pendulum
        for (auto &pendulum : pendulums)
        {
            pendulum.update();
            drawPendulum(pendulum, window);
        }

        window.display();
    }

    return 0;
}

