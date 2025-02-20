#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <iostream>
#include <vector>
#include <deque>
#include <cmath>
#include <cstdlib>
#include <ctime>

// --------------------------
// Kinematic
// --------------------------
struct Kinematic {
    sf::Vector2f position;
    sf::Vector2f velocity;
    float orientation; 
};

const float MAX_SPEED = 150.f;
const float MAX_FORCE = 50.f;

// --------------------------
// calculate vector length
// --------------------------
float vectorMagnitude(const sf::Vector2f &vec) {
    return std::sqrt(vec.x * vec.x + vec.y * vec.y);
}

// --------------------------
// vector normalize
// --------------------------
sf::Vector2f normalize(const sf::Vector2f &v) {
    float mag = vectorMagnitude(v);
    if (mag == 0) return sf::Vector2f(0.f, 0.f);
    return v / mag;
}

// --------------------------
// Boid 
// --------------------------
class Boid {
public:
    Kinematic kinematic;
    sf::Sprite sprite;
    sf::Texture texture;
    std::deque<sf::Vector2f> breadcrumbs; // 
    const int maxBreadcrumbs = 20;
    const int sampleInterval = 5;
    int frameCount = 0;

    Boid(const sf::Vector2f &startPos) {
        kinematic.position = startPos;
        float angle = (std::rand() % 360) * 3.14159265f / 180.f;
        kinematic.velocity = sf::Vector2f(std::cos(angle), std::sin(angle)) * (MAX_SPEED / 2.f);
        kinematic.orientation = angle * 180.f / 3.14159265f;

        if (!texture.loadFromFile("../assets/boid.png")) {
            std::cerr << "Error loading texture from ../assets/boid.png" << std::endl;
        }
        sprite.setTexture(texture);
        sprite.setScale(0.03f, 0.03f);
        sf::FloatRect bounds = sprite.getLocalBounds();
        sprite.setOrigin(bounds.width / 2.f, bounds.height / 2.f);
        sprite.setPosition(kinematic.position);
    }

    void update(float dt) {
        kinematic.position += kinematic.velocity * dt;
        //  orientation
        if (std::abs(kinematic.velocity.x) > 0.001f || std::abs(kinematic.velocity.y) > 0.001f) {
            kinematic.orientation = std::atan2(kinematic.velocity.y, kinematic.velocity.x) * 180.f / 3.14159265f;
        }
        sprite.setPosition(kinematic.position);
        sprite.setRotation(kinematic.orientation);

        
        frameCount++;
        if (frameCount % sampleInterval == 0) {
            breadcrumbs.push_back(kinematic.position);
            if (breadcrumbs.size() > maxBreadcrumbs)
                breadcrumbs.pop_front();
        }
    }

    void draw(sf::RenderWindow &window) {
        for (const auto &pos : breadcrumbs) {
            sf::CircleShape dot(5.f);
            //dot.setFillColor(sf::Color::Green);
            dot.setFillColor(sf::Color(0, 255, 0, 80));  
            dot.setOrigin(3.f, 3.f);
            dot.setPosition(pos);
            window.draw(dot);
        }
        window.draw(sprite);
    }
};

// --------------------------
// distance calu
// --------------------------
float distance(const sf::Vector2f &a, const sf::Vector2f &b) {
    sf::Vector2f diff = a - b;
    return std::sqrt(diff.x * diff.x + diff.y * diff.y);
}

// --------------------------
// Flocking 
// --------------------------

// Separation：
sf::Vector2f separation(Boid &boid, const std::vector<Boid*> &flock, float desiredSeparation) {
    sf::Vector2f steer(0.f, 0.f);
    int count = 0;
    for (auto other : flock) {
        if (other == &boid) continue;
        float d = distance(boid.kinematic.position, other->kinematic.position);
        if (d > 0 && d < desiredSeparation) {
            sf::Vector2f diff = boid.kinematic.position - other->kinematic.position;
            diff = normalize(diff);
            diff /= d; // 
            steer += diff;
            count++;
        }
    }
    if (count > 0)
        steer /= static_cast<float>(count);
    if (vectorMagnitude(steer) > 0) {
        steer = normalize(steer) * MAX_SPEED - boid.kinematic.velocity;
        if (vectorMagnitude(steer) > MAX_FORCE)
            steer = normalize(steer) * MAX_FORCE;
    }
    return steer;
}

// Alignment：
sf::Vector2f alignment(Boid &boid, const std::vector<Boid*> &flock, float neighborDist) {
    sf::Vector2f sum(0.f, 0.f);
    int count = 0;
    for (auto other : flock) {
        if (other == &boid) continue;
        float d = distance(boid.kinematic.position, other->kinematic.position);
        if (d > 0 && d < neighborDist) {
            sum += other->kinematic.velocity;
            count++;
        }
    }
    if (count > 0) {
        sum /= static_cast<float>(count);
        sum = normalize(sum) * MAX_SPEED;
        sf::Vector2f steer = sum - boid.kinematic.velocity;
        if (vectorMagnitude(steer) > MAX_FORCE)
            steer = normalize(steer) * MAX_FORCE;
        return steer;
    }
    return sf::Vector2f(0.f, 0.f);
}

// Cohesion：
sf::Vector2f cohesion(Boid &boid, const std::vector<Boid*> &flock, float neighborDist) {
    sf::Vector2f sum(0.f, 0.f);
    int count = 0;
    for (auto other : flock) {
        if (other == &boid) continue;
        float d = distance(boid.kinematic.position, other->kinematic.position);
        if (d > 0 && d < neighborDist) {
            sum += other->kinematic.position;
            count++;
        }
    }
    if (count > 0) {
        sum /= static_cast<float>(count);
        sf::Vector2f desired = sum - boid.kinematic.position;
        desired = normalize(desired) * MAX_SPEED;
        sf::Vector2f steer = desired - boid.kinematic.velocity;
        if (vectorMagnitude(steer) > MAX_FORCE)
            steer = normalize(steer) * MAX_FORCE;
        return steer;
    }
    return sf::Vector2f(0.f, 0.f);
}

// --------------------------
// main
// --------------------------
int main() {
    std::srand(static_cast<unsigned>(std::time(nullptr)));
    sf::RenderWindow window(sf::VideoMode(800, 600), "Flocking Behavior Demo");
    window.setFramerateLimit(60);

    // build boid
    std::vector<Boid*> flock;
    const int NUM_BOIDS = 30;
    for (int i = 0; i < NUM_BOIDS; i++) {
        float x = std::rand() % 800;
        float y = std::rand() % 600;
        flock.push_back(new Boid(sf::Vector2f(x, y)));
    }

    // 
    float sepWeight = 4.0f;
    float aliWeight = 1.0f;
    float cohWeight = 2.0f;
    float desiredSeparation = 25.f;
    float neighborDist = 50.f;

    sf::Clock clock;
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        float dt = clock.restart().asSeconds();

        for (auto boid : flock) {
            sf::Vector2f sep = separation(*boid, flock, desiredSeparation);
            sf::Vector2f ali = alignment(*boid, flock, neighborDist);
            sf::Vector2f coh = cohesion(*boid, flock, neighborDist);

            sf::Vector2f steering = sep * sepWeight + ali * aliWeight + coh * cohWeight;
            boid->kinematic.velocity += steering * dt;

            float speed = vectorMagnitude(boid->kinematic.velocity);
            if (speed > MAX_SPEED) {
                boid->kinematic.velocity = normalize(boid->kinematic.velocity) * MAX_SPEED;
            }
            boid->update(dt);

            if (boid->kinematic.position.x < 0)
                boid->kinematic.position += sf::Vector2f(800.f, 0.f);
            else if (boid->kinematic.position.x > 800)
                boid->kinematic.position -= sf::Vector2f(800.f, 0.f);
            if (boid->kinematic.position.y < 0)
                boid->kinematic.position += sf::Vector2f(0.f, 600.f);
            else if (boid->kinematic.position.y > 600)
                boid->kinematic.position -= sf::Vector2f(0.f, 600.f);
        }

        window.clear(sf::Color::White);
        for (auto boid : flock) {
            boid->draw(window);
        }
        window.display();
    }

    for (auto boid : flock)
        delete boid;

    return 0;
}
