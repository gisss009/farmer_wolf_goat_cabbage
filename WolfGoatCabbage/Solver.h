#pragma once

#include "State.h"

#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

// Показатели эффективности поиска.
//   expanded  - раскрыто узлов (для узла построены потомки);
//   generated - порождено узлов дерева поиска (включая корень);
//   rejectedUnsafe   - отброшено ходов, ведущих в недопустимое состояние;
//   rejectedRepeated - отброшено ходов, ведущих в уже встречавшееся состояние;
//   maxFrontier - максимальный размер фронта (OPEN: очередь / стек / текущий путь);
//   closedSize  - размер списка посещённых состояний (CLOSED) в конце работы;
//   maxDepth    - максимальная глубина раскрытого узла.
struct SearchStats {
    uint64_t expanded = 0;
    uint64_t generated = 0;
    uint64_t rejectedUnsafe = 0;
    uint64_t rejectedRepeated = 0;
    size_t maxFrontier = 0;
    size_t closedSize = 0;
    int maxDepth = 0;
    double timeMicroseconds = 0.0;
};

// Одна итерация IDS (поиск с ограничением глубины limit).
struct IdsIteration {
    int limit = 0;
    uint64_t expanded = 0;
    uint64_t generated = 0;
    bool found = false;
};

struct SearchResult {
    std::string algorithm;
    bool found = false;
    std::vector<State> path;   // path[0] - начальное состояние, path.back() - целевое
    std::vector<Move> moves;   // moves[i] переводит path[i] в path[i + 1]
    SearchStats stats;
    std::vector<IdsIteration> iterations;  // только для IDS
};

// Поиск в ширину (поиск на графе: состояние попадает в очередь не более
// одного раза). Проверка на цель - при порождении узла.
SearchResult SolveBFS(State start, State goal, std::ostream* trace = nullptr);

// Поиск в глубину на явном стеке со списком CLOSED (поиск на графе: раскрытое
// состояние повторно не раскрывается). Проверка на цель - при извлечении из стека.
SearchResult SolveDFS(State start, State goal, std::ostream* trace = nullptr);

// Поиск с итеративным углублением: поиск в глубину с ограничением
// limit = 0, 1, 2, ..., maxDepth. Повторы отсекаются только в пределах
// текущего пути (циклы), список CLOSED не хранится.
SearchResult SolveIDS(State start, State goal, int maxDepth = 50, std::ostream* trace = nullptr);

// Все простые (без повторения состояний) пути из start в goal -
// полный перебор в глубину. Используется для анализа пространства состояний.
std::vector<SearchResult> EnumerateAllSolutions(State start, State goal);
