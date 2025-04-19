#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include <deque>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <limits>
#include <algorithm>
#include <string>
#include <fstream>
#include <sstream>
#include <ctime>
#include <queue>
#include <utility>
#include <tuple>
using namespace std;

// ----------------------------
// const
// ----------------------------
static const float PI = 3.14159f;
static const int   CRUMB_SIZE      = 10;
static const float MAX_SPEED       = 0.5f;
static const int   NUM_BOUND_WALLS = 4;
static const int   NUM_ROOM_WALLS  = 12;
// 新增：look-ahead 距離參數，用於平滑轉換路徑節點
static const float LOOK_AHEAD_DIST = 20.f;

float vecDistance(sf::Vector2f a, sf::Vector2f b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return std::sqrt(dx*dx + dy*dy);
}

// ----------------------------
// SteeringBehaviour 
// ----------------------------
class SteeringBehaviour {
public:
    virtual void calculate_velocity(sf::Vector2f curr, sf::Vector2f target) = 0;
    virtual sf::Vector2f get_position() = 0;
    virtual void set_position(sf::Vector2i pos) = 0;
    virtual ~SteeringBehaviour() {}
};

// ----------------------------
// Position
// ----------------------------
class Position : public SteeringBehaviour {
public:
    float radiusDeceleration   = 100.f;
    float radiusSatisfaction   = 1.f;
    float timeToTargetVelocity = 10.f;
    sf::Vector2f target, initial;
    sf::Vector2f position;

    void set_position(sf::Vector2i pos) override {
        position.x = pos.x; position.y = pos.y;
    }
    sf::Vector2f get_position() override {
        return position;
    }
    void calculate_velocity(sf::Vector2f, sf::Vector2f) override {}
};

// ----------------------------
// Velocity
// ----------------------------
class Velocity : public SteeringBehaviour {
public:
    sf::Vector2f velocity, goalVelocity;
    float maxAcceleration = 1.f;

    sf::Vector2f normalize(sf::Vector2f vec){
        float d = sqrt(vec.x*vec.x + vec.y*vec.y);
        if(d>1e-6f){ vec.x/=d; vec.y/=d; }
        return vec;
    }

    void calculate_velocity(sf::Vector2f, sf::Vector2f) override {}
    sf::Vector2f get_position() override { return sf::Vector2f(0,0); }
    void set_position(sf::Vector2i) override {}
};

// ----------------------------
// Rotation
// ----------------------------
class Rotation : public SteeringBehaviour {
public:
    float rotation=0.f, maxRotation=0.3f, reqRotation=0.f;
    float rotationSatisfaction=0.0001f, rotationDeceleration=0.20f;
    float targetRotation=0.f, initialRotation=0.f;
    float timeToTargetRotation=1.5f;

    void calculate_velocity(sf::Vector2f, sf::Vector2f) override {}
    sf::Vector2f get_position() override { return sf::Vector2f(0,0); }
    void set_position(sf::Vector2i) override {}
};

// ----------------------------
// BoidSprite： Velocity, Position, Rotation
// ----------------------------
class BoidSprite : public Velocity, public Position, public Rotation {
private:
    sf::Texture texture;
public:
    bool change_room = false;
    bool vel_flag    = false;
    sf::Sprite sprite;

    BoidSprite(){
        if(!texture.loadFromFile("../assets/boid.png")){
            cerr<<"Could not load boid.png\n";
        }
        sprite.setTexture(texture);
        auto bounds = sprite.getLocalBounds();
        sprite.setOrigin(bounds.width/2, bounds.height/2);
        sprite.setScale(0.03f, 0.03f);

        velocity = sf::Vector2f(0,0);
        position = sf::Vector2f(50,50);
        sprite.setPosition(position);
    }

    sf::Vector2f get_position() override {
        return sprite.getPosition();
    }
    void set_position(sf::Vector2i pos) override {
        sprite.setPosition((float)pos.x, (float)pos.y);
    }
    void set_vel_flag(bool f){ vel_flag=f; }
    bool get_vel_flag(){ return vel_flag; }

    void move(){
        calculate_velocity(initial, target);
        if(fabs(targetRotation - rotation) > rotationSatisfaction){
            calculate_rotation();
            sprite.setRotation(rotation);
            initialRotation = rotation;
        }
        sf::Vector2f loc = get_position() + velocity;
        sprite.setPosition(loc);
        initial += velocity;
    }

    void calculate_velocity(sf::Vector2f curr, sf::Vector2f final) override {
        float goalSpeed;
        sf::Vector2f delta = final - curr;
        float dist = sqrt(delta.x*delta.x + delta.y*delta.y);

        if(dist < radiusSatisfaction){
            if(vel_flag) set_vel_flag(false);
            goalSpeed = 0; change_room = true;
        }
        else if(dist > radiusDeceleration){
            goalSpeed = MAX_SPEED;
        } else {
            goalSpeed = MAX_SPEED * dist / radiusDeceleration;
        }

        goalVelocity = normalize(delta) * goalSpeed;

        if(vel_flag){
            sf::Vector2f ngv(0,0);
            float drot = rotation - initialRotation;
            if(rotation < 0){
                ngv.x = -velocity.x*cos(drot) + velocity.y*sin(drot);
                ngv.y = -velocity.x*sin(drot) - velocity.y*cos(drot);
            } else {
                ngv.x =  velocity.x*cos(drot) - velocity.y*sin(drot);
                ngv.y =  velocity.x*sin(drot) + velocity.y*cos(drot);
            }
            ngv = normalize(ngv)*goalSpeed;
            goalVelocity = normalize(goalVelocity)*goalSpeed;
            velocity = (goalVelocity - ngv) / timeToTargetVelocity;
            target -= velocity;
            set_target_rotation(curr, target);
        } else {
            goalVelocity = normalize(goalVelocity)*goalSpeed;
            velocity = (goalVelocity - velocity) / timeToTargetVelocity;
        }
        target = final;
    }

    void set_target_rotation(sf::Vector2f curr, sf::Vector2f final){
        sf::Vector2f d = final - curr;
        reqRotation = atan2(d.y,d.x)*180.f/PI;
        int rr = (int)reqRotation; float dec=reqRotation-rr;
        reqRotation = (rr%360) + dec;
        targetRotation = reqRotation;
    }

    void calculate_rotation(){
        float gr, diff = targetRotation - rotation;
        if(fabs(diff) < rotationSatisfaction) gr = 0;
        else if(fabs(diff) > rotationDeceleration)
            gr = (diff>0?maxRotation:-maxRotation);
        else
            gr = maxRotation * diff / rotationDeceleration;
        gr /= timeToTargetRotation;
        gr = (gr<0?max(gr,-maxRotation):min(gr,maxRotation));

        int igr=(int)gr; float dec=gr-igr;
        int irot=(int)rotation; float rdec=rotation-irot;
        rotation = ((irot+igr)%360) + dec + rdec;
    }
};

// ----------------------------
// breadcrumb
// ----------------------------
class crumb : public sf::CircleShape {
public:
    crumb(){
        setRadius(5.f);
        setOrigin(2.5f,2.5f);
        setFillColor(sf::Color(0,255,0,80));
        setPosition(-100,-100);
    }
    void draw(sf::RenderWindow &win){ win.draw(*this); }
    void drop(sf::Vector2f p){ setPosition(p); }
};

// ----------------------------
// room
// ----------------------------
class Room {
public:
    int room_id;
    sf::Vector2f top_left, top_right, bottom_right, bottom_left, centre;
    vector<sf::Vector2f> room_entry;
    int num_entries;

    Room(vector<sf::Vector2f> coords, int id, vector<sf::Vector2f> entries){
        top_left     = coords[0];
        top_right    = coords[1];
        bottom_right = coords[2];
        bottom_left  = coords[3];
        room_id      = id;
        room_entry   = entries;
        num_entries  = entries.size();
        centre.x     = (top_left.x + bottom_right.x)*0.5f;
        centre.y     = (top_left.y + bottom_right.y)*0.5f;
    }
};

// ----------------------------
// A* graph
// ----------------------------
class AStarGraph {
public:
    unordered_map<int, vector<pair<int,int>>> adjacency;
    vector<sf::Vector2f> center;

    AStarGraph(int n){ center.resize(n); }
    void setCenter(int i, sf::Vector2f c){ center[i]=c; }
    void addEdge(int u,int v,int w,bool b=true){
        adjacency[u].push_back({v,w});
        if(b) adjacency[v].push_back({u,w});
    }
    float h(int a,int b){
        return vecDistance(center[a], center[b]);
    }
    vector<int> a_star(int s,int g){
        unordered_map<int,float> G,F;
        unordered_map<int,int> P;
        unordered_map<int,bool> vis;
        for(auto &kv:adjacency){
            G[kv.first]=F[kv.first]=1e9f; vis[kv.first]=false;
        }
        G[s]=0; F[s]=h(s,g);
        auto cmp=[&](auto &a,auto &b){return a.first>b.first;};
        priority_queue<pair<float,int>,vector<pair<float,int>>,decltype(cmp)> pq(cmp);
        pq.push({F[s],s});
        while(!pq.empty()){
            auto [f,u]=pq.top(); pq.pop();
            if(vis[u]) continue;
            vis[u]=true;
            if(u==g){
                vector<int> path;
                while(P.count(u)){ path.push_back(u); u=P[u]; }
                path.push_back(s);
                reverse(path.begin(),path.end());
                return path;
            }
            for(auto &e:adjacency[u]){
                int v=e.first, w=e.second;
                float alt=G[u]+w;
                if(alt<G[v]){
                    G[v]=alt; F[v]=alt+h(v,g); P[v]=u;
                    pq.push({F[v],v});
                }
            }
        }
        return {};
    }
};

// ----------------------------
// load heuristic
// ----------------------------
void loadHeuristicCSV(const string &fn, vector<sf::Vector2f> &ctr, int n){
    ctr.resize(n);
    ifstream fin(fn);
    if(!fin.is_open()){ cerr<<"Cannot open "<<fn<<"\n"; return; }
    string line; getline(fin,line);
    while(getline(fin,line)){
        if(line.empty()) continue;
        stringstream ss(line);
        int rid; float x,y;
        ss>>rid; ss.ignore(1); ss>>x; ss.ignore(1); ss>>y;
        ctr[rid]=sf::Vector2f(x,y);
    }
}

// ----------------------------
// wall
// ----------------------------
class Wall : public sf::RectangleShape {
public:
    Wall(sf::Vector2f p,sf::Vector2f s,float r){
        setFillColor(sf::Color(128,128,128));
        setPosition(p); setSize(s); setRotation(r);
    }
    void draw(sf::RenderWindow &win){ win.draw(*this); }
};

// ----------------------------
// 决策树节点
// ----------------------------
struct DecisionTree {
    int node_id;
    DecisionTree *left, *right;
    DecisionTree(int x):node_id(x),left(nullptr),right(nullptr){}
};

// ----------------------------
// main loop
// ----------------------------
int main(){
    srand((unsigned)time(NULL));
    sf::RenderWindow window(sf::VideoMode(640,480),"HW4 Part1");
    BoidSprite sprite;
    sf::Event event;

    // crumb
    deque<crumb> breadcrumbs;
    sf::Vector2f lastCrumb = sprite.get_position();

    // load csv
    const int ROOM_COUNT = 7;
    vector<sf::Vector2f> centers(ROOM_COUNT);
    loadHeuristicCSV("heuristic.csv", centers, ROOM_COUNT);

    // built a star graoh
    AStarGraph graph(ROOM_COUNT);
    for(int i=0;i<ROOM_COUNT;i++) graph.setCenter(i,centers[i]);
    graph.addEdge(0,1,1); graph.addEdge(1,2,1);
    graph.addEdge(0,3,1); graph.addEdge(1,4,1);
    graph.addEdge(3,4,1); graph.addEdge(3,5,1);
    graph.addEdge(4,6,1);

    // build wall
    sf::Vector2f bp[4]={{0,0},{640,0},{640,480},{0,480}};
    sf::Vector2f bs={640,10};
    float br[4]={0,90,180,270};
    Wall bound_walls[NUM_BOUND_WALLS]={
        Wall(bp[0],bs,br[0]),Wall(bp[1],bs,br[1]),
        Wall(bp[2],bs,br[2]),Wall(bp[3],bs,br[3])
    };


    sf::Vector2f wp[NUM_ROOM_WALLS] = {
      {210,0},{420,0},{210,100},{420,100},
      {0,160},{120,160},{335,160},{240,160},
      {240,260},{0,320},{140,320},{380,320}
    };
    sf::Vector2f ws[NUM_ROOM_WALLS] = {
      {60,10},{60,10},{60,10},{60,10},
      {80,10},{175,10},{305,10},{60,10},
      {220,10},{100,10},{200,10},{300,10}
    };
    float wr[NUM_ROOM_WALLS] = {90,90,90,90,0,0,0,90,90,0,0,0};

    vector<Wall> room_walls;
    room_walls.reserve(NUM_ROOM_WALLS);
    for(int i=0;i<NUM_ROOM_WALLS;i++){
        room_walls.emplace_back(wp[i], ws[i], wr[i]);
    }

    // build room
    vector<Room> rooms = {
        Room({{5,5},{200,5},{200,155},{5,155}},   0, {{200,80},{100,155}}),
        Room({{215,5},{415,5},{415,155},{215,155}},1, {{200,80},{415,80},{315,155}}),
        Room({{425,5},{635,5},{635,155},{425,155}},2, {{415,80}}),
        Room({{5,165},{235,165},{235,315},{5,315}},3, {{100,155},{240,240},{120,315}}),
        Room({{245,165},{635,165},{635,315},{245,315}},4, {{315,155},{240,240},{360,315}}),
        Room({{5,325},{235,325},{235,475},{5,475}},5, {{120,315}}),
        Room({{245,325},{635,325},{635,475},{245,475}},6, {{360,315}})
    };

    int current_room=0, clicked_room=0;
    bool same_room=true, travelingPath=false;
    sf::Vector2i mouse_curr, mouse_final;
    sf::Vector2f mouse_final_f;

    vector<sf::Vector2f> pathWaypoints;
    int current_waypoint_index=-1;

    // build DT
    DecisionTree* root = new DecisionTree(0);
    root->left               = new DecisionTree(3);
    root->right              = new DecisionTree(1);
    root->right->left        = new DecisionTree(3);
    root->right->right       = new DecisionTree(2);
    root->right->right->left = new DecisionTree(4);
    root->right->right->right= new DecisionTree(5);
    
// init wander
    sprite.initial = sprite.get_position();
    {
    auto &r = rooms[current_room];
    const float M = 10.f;
    sf::Vector2f nt;
    nt.x = r.top_left.x + M
         + static_cast<float>(rand())/RAND_MAX
           * (r.top_right.x - r.top_left.x - 2*M);
    nt.y = r.top_left.y + M
         + static_cast<float>(rand())/RAND_MAX
           * (r.bottom_left.y - r.top_left.y - 2*M);
    sprite.target = nt;
    sprite.set_vel_flag(true);
    sprite.calculate_velocity(sprite.initial, sprite.target);
    sprite.set_target_rotation(sprite.initial, sprite.target);
    }
// ============================

    sf::Clock clock;
    while(window.isOpen()){
        
        while(window.pollEvent(event)){
            if(event.type==sf::Event::Closed) window.close();
            if(event.type==sf::Event::MouseButtonPressed){
                mouse_curr   = mouse_final;
                mouse_final  = sf::Vector2i(event.mouseButton.x, event.mouseButton.y);
                mouse_final_f= sf::Vector2f(mouse_final);

                auto &r = rooms[current_room];
                //  seek
                if(mouse_final.x>r.top_left.x && mouse_final.x<r.top_right.x &&
                   mouse_final.y>r.top_left.y && mouse_final.y<r.bottom_left.y)
                {
                    same_room=true; travelingPath=false; pathWaypoints.clear();
                    if(mouse_final!=mouse_curr){
                        if(sprite.velocity!=sf::Vector2f(0,0)) sprite.set_vel_flag(true);
                        sprite.calculate_velocity(sprite.get_position(), mouse_final_f);
                        sprite.initial = sprite.get_position();
                        sprite.target  = mouse_final_f;
                        sprite.set_target_rotation(sprite.initial, sprite.target);
                    }
                }
                else {
                    // A* 
                    same_room=false; travelingPath=true; pathWaypoints.clear();
                    for(int i=0;i<rooms.size();i++){
                        auto &rr=rooms[i];
                        if(mouse_final.x>rr.top_left.x&&mouse_final.x<rr.top_right.x&&
                           mouse_final.y>rr.top_left.y&&mouse_final.y<rr.bottom_left.y){
                            clicked_room=rr.room_id; break;
                        }
                    }
                    auto path = graph.a_star(current_room, clicked_room);
                    if(path.empty()) travelingPath=false;
                    else {
                        
                        for(int i=0;i+1<path.size();i++){
                            int A=path[i], B=path[i+1];
                            for(auto &ea:rooms[A].room_entry){
                                for(auto &eb:rooms[B].room_entry){
                                    if(vecDistance(ea,eb)<1.f){
                                        pathWaypoints.push_back(ea);
                                        goto nextDoor;
                                    }
                                }
                            }
                            nextDoor: ;
                        }
                        pathWaypoints.push_back(mouse_final_f);
                        current_waypoint_index=0;
                        sprite.initial = sprite.get_position();
                        sprite.target  = pathWaypoints[0];
                        sprite.set_vel_flag(true);
                        sprite.calculate_velocity(sprite.initial, sprite.target);
                        sprite.set_target_rotation(sprite.initial, sprite.target);
                    }
                }
            }
        }

        //  wander
        if(!travelingPath){
            float speed = vecDistance(sprite.velocity, {0,0});
            bool atMaxSpeed = speed >= MAX_SPEED - 1e-3f;
            bool atTarget   = vecDistance(sprite.get_position(), sprite.target)
                                <= sprite.radiusSatisfaction;
            auto pos = sprite.get_position();
            auto &rm = rooms[current_room];
            bool nearWall = (pos.x - rm.top_left.x   <= 20.f ||
                             rm.top_right.x - pos.x  <= 20.f ||
                             pos.y - rm.top_left.y   <= 20.f ||
                             rm.bottom_left.y - pos.y<= 20.f);

            // DT traversal
            DecisionTree* node = root;
            while(node->left || node->right){
                if(node->node_id==0)      node = atMaxSpeed?node->left:node->right;
                else if(node->node_id==1) node = atTarget  ?node->left:node->right;
                else                       node = nearWall  ?node->left:node->right;
            }

            // leaf action
            if(node->node_id==4){
                
                travelingPath=true; pathWaypoints.clear();
                pathWaypoints.push_back(rm.centre);
                current_waypoint_index=0;
                sprite.initial=sprite.get_position();
                sprite.target = rm.centre;
                sprite.set_vel_flag(true);
                sprite.calculate_velocity(sprite.initial, sprite.target);
                sprite.set_target_rotation(sprite.initial, sprite.target);
            }
            else if(node->node_id==3){
                // wander：
                const float M=10.f;
                sf::Vector2f nt;
                nt.x = rm.top_left.x + M + rand()/(float)RAND_MAX*(rm.top_right.x - rm.top_left.x - 2*M);
                nt.y = rm.top_left.y + M + rand()/(float)RAND_MAX*(rm.bottom_left.y - rm.top_left.y - 2*M);
                sprite.initial = sprite.get_position();
                sprite.target = nt;
                sprite.set_vel_flag(true);
                sprite.calculate_velocity(sprite.initial, sprite.target);
                sprite.set_target_rotation(sprite.initial, sprite.target);
            }
            
        }

        
        if(travelingPath){
            if(current_waypoint_index >= 0 && current_waypoint_index < (int)pathWaypoints.size()){
                
                while(current_waypoint_index < (int)pathWaypoints.size()-1 &&
                      vecDistance(sprite.get_position(), pathWaypoints[current_waypoint_index]) < LOOK_AHEAD_DIST) {
                    current_waypoint_index++;
                    sprite.initial = sprite.get_position();
                    sprite.target = pathWaypoints[current_waypoint_index];
                    sprite.set_vel_flag(true);
                    sprite.calculate_velocity(sprite.initial, sprite.target);
                    sprite.set_target_rotation(sprite.initial, sprite.target);
                }
                sprite.move();
                if(current_waypoint_index == (int)pathWaypoints.size()-1 &&
                   vecDistance(sprite.get_position(), sprite.target) <= sprite.radiusSatisfaction){
                    travelingPath = false;
                    current_room = clicked_room;
                }
            } else {
                travelingPath = false;
            }
        } else {
            if(sprite.initial != sprite.target){
                sprite.move();
            }
        }
        // crumb update
        float cd = vecDistance(sprite.get_position(), lastCrumb);
        if(cd > 50.f){
            crumb c; c.drop(lastCrumb);
            lastCrumb = sprite.get_position();
            if(breadcrumbs.size() >= CRUMB_SIZE) breadcrumbs.pop_front();
            breadcrumbs.push_back(c);
        }

        // window
        window.clear(sf::Color::White);
        for(int i=0;i<NUM_BOUND_WALLS;i++) bound_walls[i].draw(window);
        for(int i=0;i<NUM_ROOM_WALLS;i++)  room_walls[i].draw(window);
        window.draw(sprite.sprite);
        for(auto &c:breadcrumbs) c.draw(window);
        window.display();
    }

    return 0;
}
