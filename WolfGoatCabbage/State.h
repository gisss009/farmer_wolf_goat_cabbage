#pragma once

// Модель задачи «Волк, коза и капуста».
//
// Состояние - четвёрка (f, w, g, c), где каждая компонента - номер берега,
// на котором находится объект: 0 - левый (исходный), 1 - правый (целевой).
//   f - крестьянин (Farmer), он же определяет положение лодки,
//   w - волк (Wolf), g - коза (Goat), c - капуста (Cabbage).
// Четвёрка хранится в 4 младших битах байта: бит i - берег объекта i.
// Всего 2^4 = 16 состояний, из них допустимы 10.

#include <cstdint>
#include <string>
#include <vector>

enum Item : int { FARMER = 0, WOLF = 1, GOAT = 2, CABBAGE = 3 };

using State = uint8_t;

constexpr int   STATE_COUNT = 16;
constexpr State START_STATE = 0b0000;  // все на левом берегу
constexpr State GOAL_STATE  = 0b1111;  // все на правом берегу

// Что крестьянин берёт с собой в лодку. Значения Wolf/Goat/Cabbage
// совпадают с номерами соответствующих битов состояния.
enum class Cargo : int { None = 0, Wolf = 1, Goat = 2, Cabbage = 3 };

constexpr Cargo ALL_CARGO[] = { Cargo::None, Cargo::Wolf, Cargo::Goat, Cargo::Cabbage };

// Оператор (действие) - одна переправа лодки.
struct Move {
    Cargo cargo = Cargo::None;
    int from = 0;  // берег отправления
};

// Применимый к состоянию оператор и его результат. Недопустимые
// результаты тоже возвращаются - чтобы трассировка могла показать,
// почему ход отброшен.
struct Transition {
    Move move;
    State to = 0;
    bool valid = false;
};

inline int Side(State s, Item i) { return (s >> i) & 1; }

// Ограничение задачи: без крестьянина волк не остаётся с козой,
// а коза - с капустой.
bool IsValid(State s);
std::string DangerReason(State s);  // почему состояние недопустимо

// Оператор применим, если груз на том же берегу, что и крестьянин.
bool CanApply(State s, Cargo c);
State Apply(State s, Cargo c);

std::vector<Transition> AllTransitions(State s);  // все применимые операторы
std::vector<Transition> Successors(State s);      // только с допустимым результатом

std::string StateCode(State s);      // "0101" - биты в порядке f w g c
std::string StateToString(State s);  // "F.G. |~~~~| .W.C"
std::string CargoShort(Cargo c);     // "F", "F+W", "F+G", "F+C"
std::string MoveToString(const Move& m);

// Разбор строки из 4 символов 0/1 или L/R (порядок f w g c).
bool ParseState(const std::string& text, State& out);
