#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>

//linear interpolation between two points 
sf::Vector2f lerp(const sf::Vector2f& start, const sf::Vector2f& end, float ratio)
{
    return start + ratio * (end - start);
}


// calculate delta of two corner, and convert from radians to degrees.
float angleDegrees(const sf::Vector2f& direction)
{
    float radians = std::atan2(direction.y, direction.x);
    return radians * 180.f / 3.14159265f;
}

int main()
{
    sf::RenderWindow window(sf::VideoMode(800, 600), "Coordinated Sprites");

    //window.setFramerateLimit(60);

    sf::Texture texture;
    if(!texture.loadFromFile("../assets/boid-sm.png"))
    {
        std::cerr << "Failed to load sprite texture!" << std::endl;
        return -1;
    }

    // Define window offset
    const float offset = 50.f;

    // Window width and height
    float winW = static_cast<float>(window.getSize().x);
    float winH = static_cast<float>(window.getSize().y);

    // 4 corners positions
    sf::Vector2f corners[4] = {
        {offset, offset},
        {winW - offset, offset},
        {winW - offset, winH - offset},
        {offset, winH - offset}
    };

    // Each "leg" of travel takes the same amount of time
    float legDuration = 3.0f;

    /*
    the 1st sprite at t=0,
    the 2nd sprite at t=1*legDuration,
    the 3rd sprite at t=2*legDuration,
    the 4th sprite at t=3*legDuration.
    th cycle end when the 4th sprite back to the start point, so total time of a cycle is 3+4 legDuration
    */
 

    float cycleDuration = 7.0f * legDuration;

    sf::Clock clock;

    // Main loop
    while (window.isOpen())
    {
        // Handle events
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                window.close();
            }
        }

        // total time
        float rawTime = clock.getElapsedTime().asSeconds();
        // relative time to total cycle time
        float t = std::fmod(rawTime, cycleDuration);

        window.clear(sf::Color::White);

        // At most 4 sprites
        for (int spriteIndex = 0; spriteIndex < 4; ++spriteIndex)
        {

            float startTime = spriteIndex * legDuration;
            float endTime   = startTime + 4 * legDuration;


            // time out of certain sprite's range
            if (t < startTime || t >= endTime)
            {
                // Not active
                continue;
            }

            // Time since this sprite started
            float localTime = t - startTime; 

            int currentLeg = static_cast<int>(localTime / legDuration); // 0,1,2,3
            float legT = (localTime - currentLeg * legDuration) / legDuration; //fraction in that leg

            int c1 = currentLeg;            // corner index
            int c2 = (currentLeg + 1) % 4;  // next corner index

            sf::Vector2f pos = lerp(corners[c1], corners[c2], legT);

            // Compute direction (for rotation)
            sf::Vector2f direction = corners[c2] - corners[c1]; 
            float rot = angleDegrees(direction);

            // Create/update sprite
            sf::Sprite sprite(texture);
            sprite.setPosition(pos);
            sprite.setRotation(rot);

            sf::FloatRect bounds = sprite.getLocalBounds();
            sprite.setOrigin(bounds.width * 0.5f, bounds.height * 0.5f);

            window.draw(sprite);
        }

        // Display the contents of the window
        window.display();
    }

    return 0;
}
