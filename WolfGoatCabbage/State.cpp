#include "State.h"

#include <cctype>

static const char ITEM_LETTER[] = { 'F', 'W', 'G', 'C' };

bool IsValid(State s) {
    const int f = Side(s, FARMER);
    const int w = Side(s, WOLF);
    const int g = Side(s, GOAT);
    const int c = Side(s, CABBAGE);
    if (w == g && f != g) return false;  // волк съест козу
    if (g == c && f != g) return false;  // коза съест капусту
    return true;
}

std::string DangerReason(State s) {
    const int f = Side(s, FARMER);
    const int g = Side(s, GOAT);
    const bool wolfEatsGoat = Side(s, WOLF) == g && f != g;
    const bool goatEatsCabbage = Side(s, CABBAGE) == g && f != g;
    if (wolfEatsGoat && goatEatsCabbage) return "волк съест козу, коза съест капусту";
    if (wolfEatsGoat) return "волк съест козу";
    if (goatEatsCabbage) return "коза съест капусту";
    return "";
}

bool CanApply(State s, Cargo c) {
    if (c == Cargo::None) return true;
    return Side(s, static_cast<Item>(c)) == Side(s, FARMER);
}

State Apply(State s, Cargo c) {
    State r = s ^ (1u << FARMER);
    if (c != Cargo::None) r ^= (1u << static_cast<int>(c));
    return r;
}

std::vector<Transition> AllTransitions(State s) {
    std::vector<Transition> result;
    for (Cargo c : ALL_CARGO) {
        if (!CanApply(s, c)) continue;
        Transition t;
        t.move = Move{ c, Side(s, FARMER) };
        t.to = Apply(s, c);
        t.valid = IsValid(t.to);
        result.push_back(t);
    }
    return result;
}

std::vector<Transition> Successors(State s) {
    std::vector<Transition> result;
    for (const Transition& t : AllTransitions(s))
        if (t.valid) result.push_back(t);
    return result;
}

std::string StateCode(State s) {
    std::string code;
    for (int i = 0; i < 4; ++i) code += static_cast<char>('0' + Side(s, static_cast<Item>(i)));
    return code;
}

std::string StateToString(State s) {
    std::string left, right;
    for (int i = 0; i < 4; ++i) {
        const bool onRight = Side(s, static_cast<Item>(i)) == 1;
        left += onRight ? '.' : ITEM_LETTER[i];
        right += onRight ? ITEM_LETTER[i] : '.';
    }
    return left + " |~~~~| " + right;
}

std::string CargoShort(Cargo c) {
    switch (c) {
    case Cargo::Wolf:    return "F+W";
    case Cargo::Goat:    return "F+G";
    case Cargo::Cabbage: return "F+C";
    default:             return "F";
    }
}

std::string MoveToString(const Move& m) {
    std::string what;
    switch (m.cargo) {
    case Cargo::Wolf:    what = "перевозит волка"; break;
    case Cargo::Goat:    what = "перевозит козу"; break;
    case Cargo::Cabbage: what = "перевозит капусту"; break;
    default:             what = "плывёт один"; break;
    }
    const char* dir = m.from == 0 ? "слева направо" : "справа налево";
    return "Крестьянин " + what + " " + dir;
}

bool ParseState(const std::string& text, State& out) {
    std::string t;
    for (char ch : text)
        if (!std::isspace(static_cast<unsigned char>(ch))) t += ch;
    if (t.size() != 4) return false;

    State s = 0;
    for (int i = 0; i < 4; ++i) {
        const char ch = static_cast<char>(std::toupper(static_cast<unsigned char>(t[i])));
        if (ch == '1' || ch == 'R') s |= (1u << i);
        else if (ch != '0' && ch != 'L') return false;
    }
    out = s;
    return true;
}
