#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include <cmath>
#include <iostream>


struct Kinematic {
    sf::Vector2f position;  
    float orientation;      
    sf::Vector2f velocity;  
    float rotation;         
};

// ----------------------------------------------------------------------
// 1. Virtual SteeringBehavior class
// ----------------------------------------------------------------------
class SteeringBehavior {
public:
    virtual void VariableMatch(Kinematic& character, const Kinematic& target, float deltaTime) = 0;
    virtual ~SteeringBehavior() {} 
};

// ----------------------------------------------------------------------
// 2. four subclass to match kinematic variables
// ----------------------------------------------------------------------

// 2.1 Position Match
class PositionMatchingBehavior : public SteeringBehavior {
public:
    void VariableMatch(Kinematic& character, const Kinematic& target, float deltaTime) override {
        character.position = target.position;
    }
};

// 2.2 Orientation Match
class OrientationMatchingBehavior : public SteeringBehavior {
public:
    void VariableMatch(Kinematic& character, const Kinematic& target, float deltaTime) override {
        character.orientation = target.orientation;
    }
};

// 2.3 Velocity Match
class VelocityMatchingBehavior : public SteeringBehavior {
public:
    void VariableMatch(Kinematic& character, const Kinematic& target, float deltaTime) override {
        character.velocity = target.velocity;
    }
};

// 2.4 Rotation Match
class RotationMatchingBehavior : public SteeringBehavior {
public:
    void VariableMatch(Kinematic& character, const Kinematic& target, float deltaTime) override {
        character.rotation = target.rotation;
    }
};

// ----------------------------------------------------------------------
// 3. Character class
// ----------------------------------------------------------------------
class Character {
public:
    Kinematic kinematic;       
    sf::Sprite sprite; 
    sf::Texture texture;

    // Only align velocity in problem 1
    VelocityMatchingBehavior velocityBehavior;

    Character() {
        // init
        kinematic.position = sf::Vector2f(100.f, 100.f);
        kinematic.orientation = 0.f;
        kinematic.velocity = sf::Vector2f(0.f, 0.f);
        kinematic.rotation = 0.f;
        
        // shape and color
        if (!texture.loadFromFile("../assets/boid.png")) {
            std::cerr << "Error loading texture from ../assets/boid.png" << std::endl;
        }
        sprite.setTexture(texture);
        // 調整縮放比例，使圖片大小適中（可依需求調整）
        sprite.setScale(0.03f, 0.03f);
        sf::FloatRect bounds = sprite.getLocalBounds();
        sprite.setOrigin(bounds.width / 2.f, bounds.height / 2.f);
        sprite.setPosition(kinematic.position);
    }


    void update(const Kinematic& targetKinematic, float deltaTime) {
        // velocity alignment
        velocityBehavior.VariableMatch(kinematic, targetKinematic, deltaTime);
        // update position
        kinematic.position += kinematic.velocity * deltaTime;
        sprite.setPosition(kinematic.position);
    }

    // boundary check
    void wrapAround(const sf::RenderWindow &window) {
        sf::Vector2u winSize = window.getSize();
        if (kinematic.position.x < 0)
            kinematic.position.x = winSize.x;
        else if (kinematic.position.x > winSize.x)
            kinematic.position.x = 0;
        if (kinematic.position.y < 0)
            kinematic.position.y = winSize.y;
        else if (kinematic.position.y > winSize.y)
            kinematic.position.y = 0;
        sprite.setPosition(kinematic.position);
    }
};

// ----------------------------------------------------------------------
// 4. main function
// ----------------------------------------------------------------------
int main() {
    sf::RenderWindow window(sf::VideoMode(800, 600), "Part 1");
    window.setFramerateLimit(60);

    Character character;

    // mouse kinematic
    Kinematic mouseKinematic;
    mouseKinematic.position = sf::Vector2f(0.f, 0.f);
    mouseKinematic.orientation = 0.f;
    mouseKinematic.velocity = sf::Vector2f(0.f, 0.f);
    mouseKinematic.rotation = 0.f;

    sf::Vector2i previousMousePos = sf::Mouse::getPosition(window);
    sf::Clock clock;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        float deltaTime = clock.restart().asSeconds();
        sf::Vector2i currentMousePos = sf::Mouse::getPosition(window);
        
        // boundary check for mouse
        sf::Vector2u winSize = window.getSize();
        if (currentMousePos.x >= 0 && currentMousePos.x < static_cast<int>(winSize.x) &&
            currentMousePos.y >= 0 && currentMousePos.y < static_cast<int>(winSize.y))
        {
            sf::Vector2f mouseVelocity(
                (currentMousePos.x - previousMousePos.x) / deltaTime,
                (currentMousePos.y - previousMousePos.y) / deltaTime
            );
            mouseKinematic.velocity = mouseVelocity;
        } 
        else 
        {
            mouseKinematic.velocity = sf::Vector2f(0.f, 0.f);
        }
        // update mouse position
        mouseKinematic.position = sf::Vector2f(currentMousePos.x, currentMousePos.y);
        previousMousePos = currentMousePos;

        // velocity alignment
        character.update(mouseKinematic, deltaTime);
        // boundary check for char
        character.wrapAround(window);

        window.clear(sf::Color::White);
        window.draw(character.sprite);
        window.display();
    }

    return 0;
}
