#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include <deque>
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
// ArriveBehavior
// ----------------------------------------------------------------------
class ArriveBehavior : public SteeringBehavior {
public:
    float slowRadius;   //  slow down when into this radius
    float arriveRadius; //  stop when into this radius
    float maxSpeed;     

    ArriveBehavior(float slowR, float arriveR, float maxS)
        : slowRadius(slowR), arriveRadius(arriveR), maxSpeed(maxS) {}

    void VariableMatch(Kinematic& character, const Kinematic& target, float deltaTime) override {
        sf::Vector2f toTarget = target.position - character.position;
        float distance = std::sqrt(toTarget.x * toTarget.x + toTarget.y * toTarget.y);
        if (distance < arriveRadius) {
            character.velocity = sf::Vector2f(0.f, 0.f);
            return;
        }
        //  slow down gradually when into slow radius
        float desiredSpeed = maxSpeed;
        if (distance < slowRadius) {
            desiredSpeed = maxSpeed * (distance / slowRadius);
        }
        //  desiredSpeed
        sf::Vector2f desiredVelocity = (toTarget / distance) * desiredSpeed;
        character.velocity = desiredVelocity;
    }
};

// ----------------------------------------------------------------------
// AlignBehavior
// ----------------------------------------------------------------------
class AlignBehavior : public SteeringBehavior {
public:
    float maxRotation;   /
    float slowRotation;  // when rotation diff is small
    float alignThreshold; 

    AlignBehavior(float maxRot, float slowRot, float threshold)
        : maxRotation(maxRot), slowRotation(slowRot), alignThreshold(threshold) {}

    void VariableMatch(Kinematic& character, const Kinematic& /*target*/, float deltaTime) override {
        // when velocity is 0, no align
        if (std::abs(character.velocity.x) < 0.001f && std::abs(character.velocity.y) < 0.001f)
            return;

        float desiredOrientation = std::atan2(character.velocity.y, character.velocity.x) * 180.f / 3.14159265f;
        float angleDiff = desiredOrientation - character.orientation;
        // normalize between [-180, 180]
        while (angleDiff > 180.f) angleDiff -= 360.f;
        while (angleDiff < -180.f) angleDiff += 360.f;
        float rotationSize = std::abs(angleDiff);
        float rotationSpeed = maxRotation;
        if (rotationSize < slowRotation) {
            rotationSpeed = maxRotation * (rotationSize / slowRotation);
        }
        if (angleDiff < 0)
            rotationSpeed = -rotationSpeed;
        character.orientation += rotationSpeed * deltaTime;

        if (std::abs(angleDiff) < alignThreshold)
            character.orientation = desiredOrientation;
    }
};

// ----------------------------------------------------------------------
// Character class
// ----------------------------------------------------------------------
class Character {
public:
    Kinematic kinematic;       
    sf::Sprite sprite;         
    sf::Texture texture;       

    Character() {
        // load image
        if (!texture.loadFromFile("../assets/boid.png")) {
            std::cerr << "Error: Could not load texture from ../assets/boid.png" << std::endl;
        }
        sprite.setTexture(texture);
        // 
        sf::FloatRect bounds = sprite.getLocalBounds();
        sprite.setOrigin(bounds.width / 2.f, bounds.height / 2.f);
        // init
        kinematic.position = sf::Vector2f(100.f, 100.f);
        kinematic.orientation = 0.f;
        kinematic.velocity = sf::Vector2f(0.f, 0.f);
        kinematic.rotation = 0.f;
        sprite.setPosition(kinematic.position);
        sprite.scale(0.04, 0.04);
    }

    //  sprite update
    void update(float deltaTime) {
        // update posi
        kinematic.position += kinematic.velocity * deltaTime;
        sprite.setPosition(kinematic.position);
        // update rotate
        sprite.setRotation(kinematic.orientation);
    }
};

// ----------------------------------------------------------------------
// main
// ----------------------------------------------------------------------
int main() {
    sf::RenderWindow window(sf::VideoMode(800, 600), "Part 2");
    window.setFramerateLimit(60);

    Character character;

    Kinematic targetKinematic = character.kinematic;

    // 2 ArriveBehavior 
    // method 1：
    ArriveBehavior arrive1(150.f, 10.f, 300.f);
    // method 2：
    ArriveBehavior arrive2(100.f, 10.f, 600.f);
    // default to use method 1
    bool useMethod1 = true;

    AlignBehavior alignBehavior(90.f, 30.f, 5.f); // 

    // 
    std::deque<sf::Vector2f> breadcrumbs;
    const int maxBreadcrumbs = 30;
    const int sampleInterval = 5; // 
    int frameCount = 0;

    sf::Clock clock;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
                targetKinematic.position = sf::Vector2f(event.mouseButton.x, event.mouseButton.y);
            }
            if (event.type == sf::Event::KeyPressed) {
                std::cout << "Key pressed: " << static_cast<int>(event.key.code) << std::endl;
            }
            // key press space to switch to different arrive method
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Space) {
                useMethod1 = !useMethod1;
                std::cout << "Switching Arrive method: " << (useMethod1 ? "Method 1" : "Method 2") << std::endl;
            }
        }

        float deltaTime = clock.restart().asSeconds();

        // update Arrive 
        if (useMethod1)
            arrive1.VariableMatch(character.kinematic, targetKinematic, deltaTime);
        else
            arrive2.VariableMatch(character.kinematic, targetKinematic, deltaTime);

        alignBehavior.VariableMatch(character.kinematic, targetKinematic, deltaTime);

        character.update(deltaTime);

        frameCount++;
        if (frameCount % sampleInterval == 0) {
            breadcrumbs.push_back(character.kinematic.position);
            if (breadcrumbs.size() > maxBreadcrumbs)
                breadcrumbs.pop_front();
        }

        window.clear(sf::Color::White);

        for (const auto& pos : breadcrumbs) {
            sf::CircleShape breadcrumbShape(6.f);
            breadcrumbShape.setFillColor(sf::Color(150, 150, 150));
            breadcrumbShape.setOrigin(3.f, 3.f);
            breadcrumbShape.setPosition(pos);
            window.draw(breadcrumbShape);
        }

        window.draw(character.sprite);

        window.display();
    }

    return 0;
}
