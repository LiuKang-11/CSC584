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
#include <memory>
#include <random>
#include <functional>

using namespace std;

//const
static const float PI = 3.14159f;
static const int   CRUMB_SIZE      = 10;
static const float MAX_SPEED       = 25.f;      // 玩家最大速度
static const float LOOK_AHEAD_DIST = 20.f;     // A* waypoint 提前切換

float vecDistance(const sf::Vector2f& a, const sf::Vector2f& b){
    return std::hypot(a.x-b.x, a.y-b.y);
}
sf::Vector2f normalize(sf::Vector2f v){
    float len = std::hypot(v.x,v.y);
    return len>1e-6f ? v/len : sf::Vector2f{0,0};
}

// room and A* 結構 
class Room {
public:
    int room_id;
    sf::Vector2f top_left, top_right, bottom_right, bottom_left, centre;
    vector<sf::Vector2f> room_entry;         
    Room(vector<sf::Vector2f> c, int id, vector<sf::Vector2f> e)
      : room_id(id), room_entry(std::move(e))
    {
        top_left=c[0]; top_right=c[1]; bottom_right=c[2]; bottom_left=c[3];
        centre = {(top_left.x+bottom_right.x)*0.5f, (top_left.y+bottom_right.y)*0.5f};
    }

    bool contains(sf::Vector2f p) const{
        return p.x >= top_left.x  && p.x <= top_right.x &&
            p.y >= top_left.y  && p.y <= bottom_left.y;
    }
};

class AStarGraph {
public:
    unordered_map<int, vector<pair<int,int>>> adj;
    vector<sf::Vector2f> center;
    explicit AStarGraph(int n){ center.resize(n); }

    void setCenter(int i,sf::Vector2f c){ center[i]=c; }
    void addEdge(int u,int v,int w,bool b=true){
        adj[u].push_back({v,w}); if(b) adj[v].push_back({u,w});
    }
    float h(int a,int b){ return vecDistance(center[a],center[b]); }

    vector<int> a_star(int s,int g){
        unordered_map<int,float> G,F; unordered_map<int,int> P;
        struct QNode{float f; int u; bool operator>(const QNode&o)const{return f>o.f;}};
        priority_queue<QNode,vector<QNode>,greater<>> pq;
        for(auto &kv:adj){ G[kv.first]=F[kv.first]=1e9f; }
        G[s]=0; F[s]=h(s,g); pq.push({F[s],s});
        unordered_map<int,bool> vis;
        while(!pq.empty()){
            auto [f,u]=pq.top(); pq.pop(); if(vis[u])continue; vis[u]=true;
            if(u==g){
                vector<int> path; while(P.count(u)){ path.push_back(u); u=P[u]; }
                path.push_back(s); reverse(path.begin(),path.end()); return path;
            }
            for(auto &[v,w]:adj[u]){
                float alt=G[u]+w; if(alt<G[v]){
                    G[v]=alt; F[v]=alt+h(v,g); P[v]=u; pq.push({F[v],v});
                }
            }
        }
        return {};
    }
};

//BoidSprite 

class SteeringBehaviour{ 
public: 
    virtual void calculate_velocity(sf::Vector2f,sf::Vector2f)=0;
    virtual sf::Vector2f get_position() const =0;
    virtual void set_position(sf::Vector2i)=0;
    virtual ~SteeringBehaviour(){} };
class Position: public SteeringBehaviour{ 
public:
    float radiusDeceleration=70.f, radiusSatisfaction=1.f, timeToTargetVelocity=10.f;
    sf::Vector2f target,initial,position;
    void set_position(sf::Vector2i p)override{position={float(p.x),float(p.y)};}
    sf::Vector2f get_position() const override {return position;}
    void calculate_velocity(sf::Vector2f,sf::Vector2f)override{}};

class Velocity: public SteeringBehaviour{
public:
    sf::Vector2f velocity,goalVelocity;
    float maxAcceleration=1.f;
    sf::Vector2f normalize(sf::Vector2f v){float d=hypot(v.x,v.y);return d>1e-6f?v/d:v;}
    void calculate_velocity(sf::Vector2f,sf::Vector2f)override{}
    sf::Vector2f get_position() const override{return {0,0};}
    void set_position(sf::Vector2i)override{}};

class Rotation: public SteeringBehaviour{
public:
    float rotation=0,maxRotation=0.8f,reqRotation=0;
    float rotationSatisfaction=1e-4f,rotationDeceleration=0.10f;
    float targetRotation=0,initialRotation=0,timeToTargetRotation=0.5f;
    void calculate_velocity(sf::Vector2f,sf::Vector2f)override{}
    sf::Vector2f get_position() const override{return {0,0};}
    void set_position(sf::Vector2i)override{}};

class BoidSprite : public Velocity, public Position, public Rotation{
public:
    float wanderMax = 25.f;      
    bool change_room=false,vel_flag=false;
    sf::Texture texture; sf::Sprite sprite;
    BoidSprite(){
        texture.loadFromFile("../assets/boid.png");
        sprite.setTexture(texture);
        sprite.setScale(0.03f,0.03f);
        auto b=sprite.getLocalBounds(); sprite.setOrigin(b.width/2,b.height/2);
        position={50,50}; 
        sprite.setPosition(position);
    }
    sf::Vector2f get_position() const override { return sprite.getPosition(); }
    void set_position(sf::Vector2i p) override { sprite.setPosition((float)p.x,(float)p.y); }
    void set_vel_flag(bool f){ vel_flag=f; } bool get_vel_flag(){ return vel_flag; }

    //calculate_velocity / set_target_rotation / calculate_rotation
       
        void calculate_velocity(sf::Vector2f curr,sf::Vector2f final) override{
        float goalSpeed;
        sf::Vector2f delta = final - curr;
        float dist = std::hypot(delta.x, delta.y);

        // vel_flag (＝wander) for max speed
        float maxV = vel_flag ? wanderMax : MAX_SPEED;

        if (dist < radiusSatisfaction)            goalSpeed = 0.f;
        else if (dist > radiusDeceleration)       goalSpeed = maxV;
        else                                       goalSpeed = maxV * dist / radiusDeceleration;


        goalVelocity=normalize(delta)*goalSpeed;
        if(vel_flag){
            sf::Vector2f ngv{0,0}; 
            float drot=rotation-initialRotation;
            if(rotation<0){
                ngv.x=-velocity.x*cos(drot*PI/180)+velocity.y*sin(drot*PI/180);
                ngv.y=-velocity.x*sin(drot*PI/180)-velocity.y*cos(drot*PI/180);
            }else{
                ngv.x= velocity.x*cos(drot*PI/180)-velocity.y*sin(drot*PI/180);
                ngv.y= velocity.x*sin(drot*PI/180)+velocity.y*cos(drot*PI/180);
            }
            ngv=normalize(ngv)*goalSpeed;
            velocity=(goalVelocity-ngv)/timeToTargetVelocity;
            target-=velocity; set_target_rotation(curr,target);
        }else{
            velocity=(goalVelocity-velocity)/timeToTargetVelocity;
        }
        target=final;
    }

    void set_target_rotation(sf::Vector2f curr, sf::Vector2f final){
        sf::Vector2f d = final - curr;
        reqRotation = atan2(d.y,d.x)*180.f/PI;
        int rr = (int)reqRotation; float dec=reqRotation-rr;
        reqRotation = (rr%360) + dec;
        targetRotation = reqRotation;
    }
    void calculate_rotation(){
        float diff=targetRotation-rotation,gr;
        if(fabs(diff)<rotationSatisfaction)gr=0;
        else if(fabs(diff)>rotationDeceleration)gr=(diff>0?maxRotation:-maxRotation);
        else gr=maxRotation*diff/rotationDeceleration;
        gr/=timeToTargetRotation; gr=std::clamp(gr,-maxRotation,maxRotation);
        int igr=int(gr); float dec=gr-igr; int ir=int(rotation); float rdec=rotation-ir;
        rotation=((ir+igr)%360)+dec+rdec;
    }
    void move(){
        
        if(vecDistance(get_position(), target) <= radiusSatisfaction){
            velocity = {0.f, 0.f};
            return;
        }

        calculate_velocity(initial, target);

        if(fabs(targetRotation - rotation) > rotationSatisfaction){
            calculate_rotation();
            sprite.setRotation(rotation);
            initialRotation = rotation;
        }
        sprite.move(velocity);
        initial += velocity;
    }
};

//breadcrumb
class crumb : public sf::CircleShape{
public:
    crumb(){ setRadius(5); setOrigin(2.5f,2.5f); setFillColor({0,255,0,80}); setPosition(-100,-100);}
    void drop(sf::Vector2f p){ setPosition(p); }
};

//Behavior Tree
enum class NodeStatus { Success, Failure, Running };
struct BTNode{ virtual ~BTNode()=default; virtual NodeStatus tick(float dt)=0; };

class ConditionNode: public BTNode{
    function<bool()> fn_;
public: explicit ConditionNode(function<bool()> f):fn_(std::move(f)){}
    NodeStatus tick(float) override{ return fn_()?NodeStatus::Success:NodeStatus::Failure; }
};
class ActionNode: public BTNode{
    function<NodeStatus(float)> fn_;
public: explicit ActionNode(function<NodeStatus(float)> f):fn_(std::move(f)){}
    NodeStatus tick(float dt) override{ return fn_(dt); }
};
class SequenceNode: public BTNode{
    vector<unique_ptr<BTNode>> ch_;
public: void add(unique_ptr<BTNode> n){ ch_.push_back(std::move(n)); }
    NodeStatus tick(float dt) override{
        for(auto&c:ch_){auto s=c->tick(dt); if(s!=NodeStatus::Success)return s;}
        return NodeStatus::Success;
    }
};
class SelectorNode: public BTNode{
    vector<unique_ptr<BTNode>> ch_;
public: void add(unique_ptr<BTNode> n){ ch_.push_back(std::move(n)); }
    NodeStatus tick(float dt) override{
        for(auto&c:ch_){auto s=c->tick(dt); if(s!=NodeStatus::Failure)return s;}
        return NodeStatus::Failure;
    }
};
class RandomSelectorNode: public BTNode{
    vector<unique_ptr<BTNode>> ch_; mt19937 rng{random_device{}()};
public: void add(unique_ptr<BTNode> n){ ch_.push_back(std::move(n)); }
    NodeStatus tick(float dt) override{
        if(ch_.empty())return NodeStatus::Failure;
        uniform_int_distribution<size_t>dist(0,ch_.size()-1);
        return ch_[dist(rng)]->tick(dt);
    }
};
class RepeaterNode: public BTNode{
    unique_ptr<BTNode> child_;
public: explicit RepeaterNode(unique_ptr<BTNode> c):child_(std::move(c)){}
    NodeStatus tick(float dt) override{
        auto s=child_->tick(dt);
        return s==NodeStatus::Success?NodeStatus::Running:s;
    }
};

//monster
class Monster{
public:
    Monster(sf::Vector2f start,
            const vector<Room>& rooms,
            AStarGraph& graph,
            const BoidSprite& player)
      : pos_(start), start_(start),
        rooms_(rooms), graph_(graph), player_(player)
    {
        shape_.setRadius(12); shape_.setOrigin(12,12);
        shape_.setFillColor(sf::Color::Red);
        current_room_ = locateRoom(pos_);
        buildTree();
    }

    void update(float dt){ 
        // 1. if stuck
        if (vecDistance(pos_, lastPos_) < 1e-2f) {
            stuckTime_ += dt;
            if (stuckTime_ > STUCK_THRESHOLD) {
                reset();                    
                stuckTime_ = 0.f;           
                return;                    
            }
        } else {
            stuckTime_ = 0.f;
        }
        lastPos_ = pos_;
        root_->tick(dt); 
        }
    void draw(sf::RenderWindow& w){ shape_.setPosition(pos_); w.draw(shape_); }
    sf::Vector2f pos(){ return pos_; }
    void reset(){ pos_=start_; traveling_=false; path_.clear(); }

private:
    //room door
    int locateRoom(sf::Vector2f p) const{
        for(auto& r:rooms_) if(r.contains(p)) return r.room_id;
        return -1;
    }
    void buildDoorPath(int from,int to,vector<int>& roomSeq){
        auto path = graph_.a_star(from,to);
        if(path.empty()) return;
        for(size_t i=0;i+1<path.size();++i){
            int A=path[i],B=path[i+1];
            for(auto &ea:rooms_[A].room_entry)
                for(auto &eb:rooms_[B].room_entry)
                    if(vecDistance(ea,eb)<1.f){ path_.push_back(ea); goto next; }
            next:;
        }
    }
    void recalcPath(){
        path_.clear(); traveling_=false; wp_ = 0;
        int me=locateRoom(pos_), pl=locateRoom(player_.get_position());
        if(me==-1||pl==-1) return;
        buildDoorPath(me,pl,pathRooms_);
        path_.push_back(player_.get_position());
        traveling_=true; wp_=0;
    }

    //Behavior Tree  
    NodeStatus chase(float dt){
        int plRoom = locateRoom(player_.get_position());
        if(!traveling_ || plRoom!=targetRoom_){ targetRoom_=plRoom; recalcPath(); }
        followPath(dt);
        return NodeStatus::Running;
    }
    NodeStatus followPath(float dt){
        if(!traveling_) return NodeStatus::Success;
        if(wp_>=path_.size()){ traveling_=false; return NodeStatus::Success; }
        auto target = path_[wp_];
        sf::Vector2f dir = normalize(target-pos_);
        pos_ += dir*speed_*dt;

    
        if(vecDistance(pos_,target)<30.f) ++wp_;
        static float stuckTimer = 0.f;
        if(wpProgress_ == wp_)     stuckTimer += dt;
        else{ wpProgress_ = wp_;   stuckTimer = 0.f; }
        if(stuckTimer > 1.f){ recalcPath(); stuckTimer = 0.f; }
        return NodeStatus::Running;
    }
    NodeStatus dance(float dt){
        osc_t_+=dt;
        pos_.x = start_.x + std::cos(osc_t_*3)*8.f;
        pos_.y = start_.y + std::sin(osc_t_*3)*4.f;
        if(osc_t_>4.f){ osc_t_=0.f; return NodeStatus::Success; }
        return NodeStatus::Running;
    }
    NodeStatus patrol(float dt){
        if(!patrolling_){
            int me = locateRoom(pos_);
            if(me==-1) return NodeStatus::Failure;
            auto &r = rooms_[me];
            const float M=15.f;
            patrolTarget_ = { r.top_left.x+M+rand()/(float)RAND_MAX*(r.top_right.x-r.top_left.x-2*M),
                              r.top_left.y+M+rand()/(float)RAND_MAX*(r.bottom_left.y-r.top_left.y-2*M) };
            patrolling_=true;
        }
        sf::Vector2f dir = normalize(patrolTarget_-pos_);
        pos_ += dir*speed_*dt;
        if(vecDistance(pos_,patrolTarget_)<5.f){ patrolling_=false; return NodeStatus::Success; }
        return NodeStatus::Running;
    }

    void buildTree(){
        auto seqHunt = make_unique<SequenceNode>();
        seqHunt->add(make_unique<ConditionNode>([this]{
            return vecDistance(pos_,player_.get_position())<400.f;  // 偵測
        }));
        seqHunt->add(make_unique<ActionNode>([this](float dt){ return chase(dt); }));

        auto randSel = make_unique<RandomSelectorNode>();
        randSel->add(make_unique<ActionNode>([this](float dt){ return patrol(dt); }));
        randSel->add(make_unique<RepeaterNode>(
                     make_unique<ActionNode>([this](float dt){ return dance(dt); })));

        auto root = make_unique<SelectorNode>();
        root->add(std::move(seqHunt));
        root->add(std::move(randSel));
        root_ = std::move(root);
    }

private:
    sf::Vector2f pos_, start_;
    sf::CircleShape shape_;
    const vector<Room>& rooms_;
    AStarGraph& graph_;
    const BoidSprite& player_;
    float speed_{70.f};
    sf::Vector2f lastPos_;
    float stuckTime_ = 0.f;
    const float STUCK_THRESHOLD = 1.0f;  // 
    //chase path
    vector<sf::Vector2f> path_;  size_t wp_{0}, wpProgress_{0};
    bool traveling_{false};      int targetRoom_{-1}; vector<int> pathRooms_;

    //patrol
    bool patrolling_{false}; sf::Vector2f patrolTarget_;

    //wander
    float osc_t_{0.f};

    unique_ptr<BTNode> root_;
    int current_room_{0};
};

int locateRoom(const std::vector<Room>& rooms, sf::Vector2f p){
    for(const auto& r : rooms) if(r.contains(p)) return r.room_id;
    return -1;   // 不在任何房間
}

//main loop
int main(){
    srand((unsigned)time(nullptr));
    sf::RenderWindow window(sf::VideoMode(640,480),"HW4 Part2");
    window.setFramerateLimit(60);

    //build room
    const int ROOM_COUNT=7;
    vector<Room> rooms = {
        Room({{5,5},{200,5},{200,155},{5,155}},   0, {{200,80},{100,155}}),
        Room({{215,5},{415,5},{415,155},{215,155}},1, {{200,80},{415,80},{315,155}}),
        Room({{425,5},{635,5},{635,155},{425,155}},2, {{415,80}}),
        Room({{5,165},{235,165},{235,315},{5,315}},3, {{100,155},{240,240},{120,315}}),
        Room({{245,165},{635,165},{635,315},{245,315}},4, {{315,155},{240,240},{360,315}}),
        Room({{5,325},{235,325},{235,475},{5,475}},5, {{120,315}}),
        Room({{245,325},{635,325},{635,475},{245,475}},6, {{360,315}})
    };

    //A*
    AStarGraph graph(ROOM_COUNT);
    for(auto&r:rooms) graph.setCenter(r.room_id,r.centre);
    graph.addEdge(0,1,1); graph.addEdge(1,2,1);
    graph.addEdge(0,3,1); graph.addEdge(1,4,1);
    graph.addEdge(3,4,1); graph.addEdge(3,5,1);
    graph.addEdge(4,6,1);

    //wall
    auto makeWall=[&](sf::Vector2f pos,sf::Vector2f size,float rot){
        sf::RectangleShape w; w.setFillColor({128,128,128});
        w.setPosition(pos); w.setSize(size); w.setRotation(rot); return w; };
    vector<sf::RectangleShape> bound_walls, room_walls;
    bound_walls.push_back(makeWall({0,0},{640,10},0));
    bound_walls.push_back(makeWall({640,0},{480,10},90));
    bound_walls.push_back(makeWall({640,480},{640,10},180));
    bound_walls.push_back(makeWall({0,480},{480,10},270));
    sf::Vector2f wp[12]={{210,0},{420,0},{210,100},{420,100},{0,160},{120,160},{335,160},{240,160},{240,260},{0,320},{140,320},{380,320}};
    sf::Vector2f ws[12]={{60,10},{60,10},{60,10},{60,10},{80,10},{175,10},{305,10},{60,10},{220,10},{100,10},{200,10},{300,10}};
    float wr[12]={90,90,90,90,0,0,0,90,90,0,0,0};
    for(int i=0;i<12;++i) room_walls.push_back(makeWall(wp[i],ws[i],wr[i]));

    //player
    BoidSprite player;
    player.initial = player.get_position();
    { // init wander target
        auto &r=rooms[0]; const float M=10;
        player.target={ r.top_left.x+M+rand()/float(RAND_MAX)*(r.top_right.x-r.top_left.x-2*M),
                        r.top_left.y+M+rand()/float(RAND_MAX)*(r.bottom_left.y-r.top_left.y-2*M)};
        player.set_vel_flag(true); player.calculate_velocity(player.initial,player.target);
        player.set_target_rotation(player.initial,player.target);
    }

    //monster
    Monster monster({300,200},rooms,graph,player);

    //breadcrumb
    deque<crumb> breadcrumbs; sf::Vector2f lastCrumb=player.get_position();

    //main fxn
    sf::Clock clk;
    while(window.isOpen()){
        sf::Event ev; while(window.pollEvent(ev))
            if(ev.type==sf::Event::Closed) window.close();

        float dt=clk.restart().asSeconds();

        //player wander and A*
        static std::mt19937 rng{std::random_device{}()};

        // follow path 
        static bool pTraveling = false;
        static std::vector<sf::Vector2f> pWaypoints;
        static size_t pWP = 0, pDestRoom = 0;

        auto startNewWanderInRoom = [&](int roomId){

            const float M = 10.f;
            auto &r = rooms[roomId];
            sf::Vector2f nt;
            do{
                nt = {
                    r.top_left.x  + M + rng()/(float)rng.max() * (r.top_right.x  - r.top_left.x  - 2*M),
                    r.top_left.y  + M + rng()/(float)rng.max() * (r.bottom_left.y - r.top_left.y - 2*M)
                };
            } while(
                vecDistance(nt, r.centre) < 35.f           /* avoid center ±35 */
                || vecDistance(nt, player.get_position()) < 40.f );  /* target <40 */




            player.initial = player.get_position();
            player.target  = nt;
            player.set_vel_flag(true);
            player.calculate_velocity(player.initial, player.target);
            player.set_target_rotation(player.initial, player.target);
            pTraveling = false;
        };

        if(pTraveling){
            /* waypoint list */
            if(pWP < pWaypoints.size()){
                if(vecDistance(player.get_position(), pWaypoints[pWP]) < LOOK_AHEAD_DIST){
                    ++pWP;
                        if(pWP < pWaypoints.size()){
                        player.initial = player.get_position();
                        player.target  = pWaypoints[pWP];
                        player.set_vel_flag(true);
                        player.calculate_velocity(player.initial, player.target);
                        player.set_target_rotation(player.initial, player.target);
                    }
                }
            }
            player.move();
            /*  last waypoint → end traveling */
            if(pWP >= pWaypoints.size()){
                int roomNow = locateRoom(rooms, player.get_position());
                startNewWanderInRoom(roomNow >=0 ? roomNow : 0);
            }
        }
        else{
            /* wandering */
            float distToTarget = vecDistance(player.get_position(), player.target);
            float speedMag     = std::hypot(player.velocity.x, player.velocity.y);
            if(distToTarget <= player.radiusSatisfaction || speedMag < 0.1f){
                int curRoom = locateRoom(rooms, player.get_position());
                std::uniform_real_distribution<float> uni(0.f,1.f);
                if(curRoom>=0 && uni(rng) < 0.40f){      // 40% change room
                    //random choose
                    std::uniform_int_distribution<int> pick(0, ROOM_COUNT-1);
                    int newRoom;
                    do{ newRoom = pick(rng); } while(newRoom == curRoom);

                    // A* find path
                    auto seq = graph.a_star(curRoom, newRoom);
                    if(seq.size() >= 2){
                        pWaypoints.clear();
                        for(size_t i=0;i+1<seq.size();++i){
                                int A = seq[i], B = seq[i+1];
                                for(auto &ea : rooms[A].room_entry)
                                for(auto &eb : rooms[B].room_entry)
                                    if(vecDistance(ea,eb)<1.f){ pWaypoints.push_back(ea); goto nextDoor; }
                        nextDoor:;
                        }
                        
                        const float M = 10.f;
                        auto &nr = rooms[newRoom];
                        sf::Vector2f rnd{
                            nr.top_left.x+M+rng()/(float)rng.max()*(nr.top_right.x-nr.top_left.x-2*M),
                            nr.top_left.y+M+rng()/(float)rng.max()*(nr.bottom_left.y-nr.top_left.y-2*M)};
                        if(vecDistance(rnd, nr.centre) < 20.f){
                            rnd.x += (rnd.x < nr.centre.x ? -20.f : 20.f);
                            rnd.y += (rnd.y < nr.centre.y ? -20.f : 20.f);
                        }
                        pWaypoints.push_back(rnd);

                        pWP = 0; pTraveling = true;

                        player.initial = player.get_position();
                        player.target  = pWaypoints[0];
                        player.set_vel_flag(true);
                        player.calculate_velocity(player.initial, player.target);
                        player.set_target_rotation(player.initial, player.target);
                    }
                    else{
                        startNewWanderInRoom(curRoom);   // Fallback 
                    }
                }
                else{
                    startNewWanderInRoom(curRoom>=0?curRoom:0);  //  wander
                }
            }
            else{
                player.move();     
            }
        }




        /* Monster BT */
        monster.update(dt);

        /* collision and reset */
        if(vecDistance(player.get_position(),monster.pos())<24.f){
            
            player.set_position({int(rooms[0].centre.x), int(rooms[0].centre.y)});
            player.velocity = {0.f, 0.f};                      
            player.initial  = player.get_position();
            player.target   = player.get_position();

            pTraveling = false;
            pWaypoints.clear();
            startNewWanderInRoom(0);                          
            monster.reset();
         }

        /* breadcrumb */
        if(vecDistance(player.get_position(),lastCrumb)>50){
            crumb c; c.drop(lastCrumb); breadcrumbs.push_back(c);
            if(breadcrumbs.size()>CRUMB_SIZE) breadcrumbs.pop_front();
            lastCrumb=player.get_position();
        }

        /* Render */
        window.clear(sf::Color::White);
        for(auto&w:bound_walls) window.draw(w);
        for(auto&w:room_walls)  window.draw(w);
        window.draw(player.sprite);
        monster.draw(window);
        for(auto&c:breadcrumbs) window.draw(c);
        window.display();
    }
    return 0;
}
