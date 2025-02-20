#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include <deque>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>

// --------------------------
// Kinematic
// --------------------------
struct Kinematic {
    sf::Vector2f position;
    float orientation;      
    sf::Vector2f velocity;
    float rotation;
};

// --------------------------
// SteeringBehavior
// --------------------------
class SteeringBehavior {
public:
    virtual void update(Kinematic &character, float deltaTime) = 0;
    virtual ~SteeringBehavior() {}
};

// --------------------------
// WanderBehavior
// --------------------------
class WanderBehavior : public SteeringBehavior {
public:
    // 
    float wanderOffset;   
    float wanderRadius;   
    float wanderRate;     
    float wanderAngle;    
    bool useMethod1;      
    float fixedSpeed;     

    WanderBehavior(float offset, float radius, float rate, float speed, bool method1 = true)
        : wanderOffset(offset), wanderRadius(radius), wanderRate(rate),
          wanderAngle(0.f), useMethod1(method1), fixedSpeed(speed) {}

    // update
    void update(Kinematic &character, float deltaTime) override {
        if (useMethod1) {
            // 1：Circle Wander
            float randomChange = (((float)std::rand() / RAND_MAX) - 0.5f) * 2 * wanderRate * deltaTime;
            wanderAngle += randomChange;
            sf::Vector2f forward;
            if (std::abs(character.velocity.x) < 0.001f && std::abs(character.velocity.y) < 0.001f) {
                float theta = character.orientation * 3.14159265f / 180.f;
                forward = sf::Vector2f(std::cos(theta), std::sin(theta));
            } else {
                float mag = std::sqrt(character.velocity.x * character.velocity.x +
                                      character.velocity.y * character.velocity.y);
                forward = character.velocity / mag;
            }
            sf::Vector2f circleCenter = forward * wanderOffset;
            sf::Vector2f displacement(wanderRadius * std::cos(wanderAngle), wanderRadius * std::sin(wanderAngle));
            sf::Vector2f wanderForce = circleCenter + displacement;
            float magForce = std::sqrt(wanderForce.x * wanderForce.x + wanderForce.y * wanderForce.y);
            if(magForce != 0)
                wanderForce /= magForce;
            character.velocity = wanderForce * fixedSpeed;
            character.orientation = std::atan2(character.velocity.y, character.velocity.x) * 180.f / 3.14159265f;
        } else {
            // 2：Jitter Wander

            float randomJitter = (((float)std::rand() / RAND_MAX) - 0.5f) * 2 * wanderRate * deltaTime * 180.f / 3.14159265f;
            character.orientation += randomJitter;
            float theta = character.orientation * 3.14159265f / 180.f;
            sf::Vector2f dir(std::cos(theta), std::sin(theta));
            character.velocity = dir * fixedSpeed;
        }
        character.position += character.velocity * deltaTime;
    }
};

// --------------------------
// Character 
// --------------------------
class Character {
public:
    Kinematic kinematic;
    sf::Sprite sprite;
    sf::Texture texture;

    Character() {
        if (!texture.loadFromFile("../assets/boid.png")) {
            std::cerr << "Error: Could not load texture from ../assets/boid.png" << std::endl;
        }
        sprite.setTexture(texture);
        sprite.setScale(0.1f, 0.1f);

        sf::FloatRect bounds = sprite.getLocalBounds();
        sprite.setOrigin(bounds.width / 2.f, bounds.height / 2.f);
        // 
        kinematic.position = sf::Vector2f(400.f, 300.f);
        kinematic.orientation = 0.f;
        // 
        kinematic.velocity = sf::Vector2f(200.f, 0.f);
        kinematic.rotation = 0.f;
        sprite.setPosition(kinematic.position);
        sprite.scale(0.4, 0.4);

    }
    void update() {
        sprite.setPosition(kinematic.position);
        sprite.setRotation(kinematic.orientation);
    }
};

// --------------------------
// main
// --------------------------
int main() {
    std::srand(std::time(nullptr));
    sf::RenderWindow window(sf::VideoMode(800, 600), "Wander Steering Behaviors Demo");
    window.setFramerateLimit(60);

    Character character;
    //  1 (Circle Wander)
    WanderBehavior wander(100.f, 50.f, 8.0f, 200.f, true);
    bool useMethod1 = true;

    std::deque<sf::Vector2f> breadcrumbs;
    const int maxBreadcrumbs = 30;
    const int sampleInterval = 5;
    int frameCount = 0;

    sf::Clock clock;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
            // switch wander method
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Space) {
                useMethod1 = !useMethod1;
                wander.useMethod1 = useMethod1;
                std::cout << "Switched wander method: " 
                          << (useMethod1 ? "Method 1 (Circle Wander)" : "Method 2 (Jitter Wander)") 
                          << std::endl;
            }
        }

        float deltaTime = clock.restart().asSeconds();

        wander.update(character.kinematic, deltaTime);

        if (character.kinematic.position.x < 0)
            character.kinematic.position.x = 800;
        else if (character.kinematic.position.x > 800)
            character.kinematic.position.x = 0;
        if (character.kinematic.position.y < 0)
            character.kinematic.position.y = 600;
        else if (character.kinematic.position.y > 600)
            character.kinematic.position.y = 0;

        character.update();

        frameCount++;
        if (frameCount % sampleInterval == 0) {
            breadcrumbs.push_back(character.kinematic.position);
            if (breadcrumbs.size() > maxBreadcrumbs)
                breadcrumbs.pop_front();
        }

        window.clear(sf::Color::White);
        for (const auto &pos : breadcrumbs) {
            sf::CircleShape crumb(6.f);
            //crumb.setFillColor(sf::Color::Blue);
            crumb.setFillColor(sf::Color(0, 0, 255, 128));  // 

            crumb.setOrigin(3.f, 3.f);
            crumb.setPosition(pos);
            window.draw(crumb);
        }
        window.draw(character.sprite);
        window.display();
    }

    return 0;
}
