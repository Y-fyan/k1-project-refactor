#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <array>

// HSL 2026 v20。Brain 和 UDP 节点共用定义，ROS 消息不作为网络内存布局。
namespace gc {
const unsigned dataPort = 3838, returnPort = 3939, maxPlayers = 20;
const unsigned packetSize = 158, returnSize = 32;
enum State { Initial, Ready, Set, Playing, Finished };
enum Phase { Normal, PenaltyShootout, ExtraTime, Timeout };
enum SetPlay { None, DirectFreeKick, IndirectFreeKick, PenaltyKick, ThrowIn, GoalKick, CornerKick };
const unsigned noTeam = 255, substitute = 13;
struct Player { std::uint8_t penalty = 0, secsTillUnpenalised = 0, cautions = 0; };
struct Team {
    std::uint8_t number = 0, fieldColour = 0, keeperColour = 0, goalkeeper = 0, score = 0, penaltyShot = 0;
    std::uint16_t singleShots = 0, messageBudget = 0;
    Player players[maxPlayers];
};
struct Packet {
    char header[4] = {'R','G','m','e'};
    std::uint8_t version = 20, number = 0, playersPerTeam = 0, competitionType = 0;
    std::uint8_t stopped = 0, phase = 0, state = 0, setPlay = 0, firstHalf = 0, kickingTeam = noTeam;
    std::int16_t secsRemaining = 0, secondaryTime = 0;
    Team teams[2];
};
struct ReturnData {
    char header[4] = {'R','G','r','t'};
    std::uint8_t version = 4, player = 0, team = 0, fallen = 1;
    float pose[3] = {0,0,0}, ballAge = -1, ball[2] = {0,0};
};
static_assert(sizeof(Packet) == packetSize, "v20 layout");
static_assert(sizeof(ReturnData) == returnSize, "v4 layout");
inline bool containsTeam(const Packet& p, unsigned team) { return p.teams[0].number == team || p.teams[1].number == team; }
inline bool valid(const Packet& p) {
    if(std::memcmp(p.header,"RGme",4) || p.version != 20 || !p.playersPerTeam || p.playersPerTeam > maxPlayers ||
       p.competitionType > 2 || p.stopped > 1 || p.phase > Timeout || p.state > Finished || p.setPlay > CornerKick ||
       p.firstHalf > 1 || p.teams[0].number == p.teams[1].number ||
       (p.kickingTeam != noTeam && !containsTeam(p,p.kickingTeam))) return false;
    for(const Team& t : p.teams) {
        if(t.fieldColour > 9 || t.keeperColour > 9 || t.goalkeeper > maxPlayers) return false;
        for(const Player& player : t.players) if(player.penalty > substitute) return false;
    }
    return true;
}
inline std::uint16_t u16(const std::uint8_t* p) { return static_cast<std::uint16_t>(p[0] | (p[1] << 8)); }
inline bool decode(const std::uint8_t* bytes, std::size_t size, Packet& p) {
    if(size != packetSize || std::memcmp(bytes,"RGme",4) || bytes[4] != 20) return false;
    p = Packet();
    p.number=bytes[5]; p.playersPerTeam=bytes[6]; p.competitionType=bytes[7]; p.stopped=bytes[8];
    p.phase=bytes[9]; p.state=bytes[10]; p.setPlay=bytes[11]; p.firstHalf=bytes[12]; p.kickingTeam=bytes[13];
    // 显式符号扩展，不依赖实现对超范围 unsigned -> signed 的处理。
    const int remaining=u16(bytes+14), secondary=u16(bytes+16);
    p.secsRemaining=static_cast<std::int16_t>(remaining >= 32768 ? remaining-65536 : remaining);
    p.secondaryTime=static_cast<std::int16_t>(secondary >= 32768 ? secondary-65536 : secondary);
    unsigned n=18;
    for(Team& t : p.teams) {
        t.number=bytes[n++]; t.fieldColour=bytes[n++]; t.keeperColour=bytes[n++]; t.goalkeeper=bytes[n++];
        t.score=bytes[n++]; t.penaltyShot=bytes[n++]; t.singleShots=u16(bytes+n); n+=2; t.messageBudget=u16(bytes+n); n+=2;
        for(Player& player : t.players) { player.penalty=bytes[n++]; player.secsTillUnpenalised=bytes[n++]; player.cautions=bytes[n++]; }
    }
    return valid(p);
}
inline bool valid(const ReturnData& p) {
    if(!p.team || !p.player || p.player > maxPlayers || p.fallen > 1 || !std::isfinite(p.ballAge) || (p.ballAge < 0 && p.ballAge != -1)) return false;
    for(float f : p.pose) if(!std::isfinite(f)) return false;
    for(float f : p.ball) if(!std::isfinite(f)) return false;
    return true;
}
inline std::array<std::uint8_t,returnSize> encode(const ReturnData& p) {
    static_assert(sizeof(float)==4, "GC requires float32");
    std::array<std::uint8_t,returnSize> bytes{};
    std::memcpy(bytes.data(),"RGrt",4); bytes[4]=4; bytes[5]=p.player; bytes[6]=p.team; bytes[7]=p.fallen;
    const float values[6]={p.pose[0],p.pose[1],p.pose[2],p.ballAge,p.ball[0],p.ball[1]};
    for(unsigned i=0;i<6;++i) {
        std::uint32_t bits; std::memcpy(&bits,&values[i],4);
        for(unsigned j=0;j<4;++j) bytes[8+i*4+j]=static_cast<std::uint8_t>(bits >> (j*8));
    }
    return bytes;
}
// 一段有效接收流的序号；断流两秒后重同步，重复包不会延长窗口。
class Sequence {
    int previous_ = -1;
    double acceptedAt_ = 0;
public:
    void reset() { previous_=-1; }
    bool accept(std::uint8_t number, double now) {
        const unsigned delta=static_cast<std::uint8_t>(number-previous_);
        if(previous_ >= 0 && now >= acceptedAt_ && now-acceptedAt_ <= 2 && (!delta || delta > 127)) return false;
        previous_=number; acceptedAt_=now; return true;
    }
};
}
