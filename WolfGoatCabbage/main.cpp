#include "State.h"
#include "Solver.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <chrono>
#include <cctype>
#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {

struct Settings {
    State start = START_STATE;
    State goal = GOAL_STATE;
    bool trace = true;
    int dfsTreeDepthLimit = 11;  // для DFS без контроля повторов
};

std::string Trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

std::string ReadLine(const std::string& prompt) {
    std::cout << prompt;
    std::string line;
    if (!std::getline(std::cin, line)) return "0";
    return Trim(line);
}

void PrintLegend() {
    std::cout << "Обозначения: F - крестьянин, W - волк, G - коза, C - капуста.\n"
                 "Состояние (f,w,g,c): 0 - объект на левом берегу, 1 - на правом.\n"
                 "Картинка \"LLLL |~~~~| RRRR\": слева - левый берег, справа - правый, '.' - объекта нет.\n";
}

// Среднее время одного запуска (без трассировки). Маленькая задача решается
// за микросекунды, поэтому запуск повторяется, пока не наберётся ~0.2 с.
double MeasureMicroseconds(const std::function<void()>& run) {
    using Clock = std::chrono::steady_clock;
    const auto budget = std::chrono::milliseconds(200);
    int reps = 0;
    const auto t0 = Clock::now();
    auto t1 = t0;
    do {
        run();
        ++reps;
        t1 = Clock::now();
    } while (t1 - t0 < budget && reps < 1'000'000);
    return std::chrono::duration<double, std::micro>(t1 - t0).count() / reps;
}

void PrintSolution(const SearchResult& r) {
    std::cout << "  " << std::setw(2) << 0 << ". " << "[" << StateCode(r.path[0]) << "] "
              << StateToString(r.path[0]) << "   (начальное состояние)\n";
    for (size_t i = 0; i < r.moves.size(); ++i) {
        std::cout << "  " << std::setw(2) << i + 1 << ". " << "[" << StateCode(r.path[i + 1]) << "] "
                  << StateToString(r.path[i + 1]) << "   " << MoveToString(r.moves[i]) << "\n";
    }
}

void PrintStats(const SearchResult& r) {
    const SearchStats& s = r.stats;
    std::cout << "Раскрыто узлов:                 " << s.expanded << "\n"
              << "Порождено узлов:                " << s.generated << "\n"
              << "Отброшено недопустимых ходов:   " << s.rejectedUnsafe << "\n"
              << "Отброшено повторных состояний:  " << s.rejectedRepeated << "\n"
              << "Макс. размер OPEN (память):     " << s.maxFrontier << "\n"
              << "Размер CLOSED:                  " << s.closedSize << "\n"
              << "Макс. глубина раскрытия:        " << s.maxDepth << "\n"
              << "Среднее время поиска:           " << std::fixed << std::setprecision(3)
              << s.timeMicroseconds << " мкс\n";
    std::cout.unsetf(std::ios::fixed);

    if (!r.iterations.empty()) {
        std::cout << "\nИтерации IDS:\n"
                  << "   L | раскрыто | порождено | результат\n"
                  << "  ---+----------+-----------+-----------\n";
        for (const IdsIteration& it : r.iterations)
            std::cout << "  " << std::setw(2) << it.limit << " | " << std::setw(8) << it.expanded << " | "
                      << std::setw(9) << it.generated << " | " << (it.found ? "найдено" : "отсечение") << "\n";
    }
}

void PrintResult(const SearchResult& r) {
    std::cout << "\n===== Результат: " << r.algorithm << " =====\n";
    if (!r.found) {
        std::cout << "Решение не найдено.\n";
    } else {
        std::cout << "Решение найдено, число переправ: " << r.moves.size() << "\n";
        PrintSolution(r);
    }
    std::cout << "\n";
    PrintStats(r);
    std::cout << "\n";
}

SearchResult RunBFS(const Settings& cfg, std::ostream* trace) { return SolveBFS(cfg.start, cfg.goal, trace); }
SearchResult RunDFS(const Settings& cfg, std::ostream* trace) { return SolveDFS(cfg.start, cfg.goal, true, -1, trace); }
SearchResult RunDFSTree(const Settings& cfg, std::ostream* trace) {
    return SolveDFS(cfg.start, cfg.goal, false, cfg.dfsTreeDepthLimit, trace);
}
SearchResult RunIDS(const Settings& cfg, std::ostream* trace) { return SolveIDS(cfg.start, cfg.goal, 50, trace); }

using Runner = SearchResult (*)(const Settings&, std::ostream*);

SearchResult RunTimed(Runner run, const Settings& cfg, bool trace) {
    if (trace) std::cout << "\n----- Трассировка поиска -----\n";
    SearchResult r = run(cfg, trace ? &std::cout : nullptr);
    r.stats.timeMicroseconds = MeasureMicroseconds([&] { run(cfg, nullptr); });
    return r;
}

void RunAndPrint(Runner run, const Settings& cfg) {
    PrintResult(RunTimed(run, cfg, cfg.trace));
}

void PrintStateSpace() {
    std::cout << "\n===== Пространство состояний =====\n";
    PrintLegend();
    std::cout << "\n  код  | картинка         | допустимо\n"
              << "  -----+------------------+---------------------------------\n";
    int validCount = 0;
    for (int i = 0; i < STATE_COUNT; ++i) {
        const State s = static_cast<State>(i);
        const bool ok = IsValid(s);
        validCount += ok;
        std::cout << "  " << StateCode(s) << " | " << StateToString(s) << " | "
                  << (ok ? "да" : "нет (" + DangerReason(s) + ")") << "\n";
    }
    std::cout << "\nВсего состояний: " << STATE_COUNT << ", допустимых: " << validCount << "\n";

    std::cout << "\nГраф переходов (только допустимые состояния):\n";
    int edges = 0;
    for (int i = 0; i < STATE_COUNT; ++i) {
        const State s = static_cast<State>(i);
        if (!IsValid(s)) continue;
        std::cout << "  " << StateCode(s) << " " << StateToString(s) << "  ->";
        for (const Transition& t : Successors(s)) {
            std::cout << "  " << CargoShort(t.move.cargo) << ":" << StateCode(t.to);
            ++edges;
        }
        std::cout << "\n";
    }
    std::cout << "Рёбер (переходы обратимы, каждое ребро учтено дважды): " << edges / 2 << "\n\n";
}

void PrintAllSolutions(const Settings& cfg) {
    const auto all = EnumerateAllSolutions(cfg.start, cfg.goal);
    std::cout << "\n===== Все простые пути из " << StateCode(cfg.start) << " в " << StateCode(cfg.goal)
              << " =====\n";
    std::cout << "Найдено путей: " << all.size() << "\n";
    for (size_t i = 0; i < all.size(); ++i) {
        std::cout << "\nРешение " << i + 1 << " (" << all[i].moves.size() << " переправ):\n";
        PrintSolution(all[i]);
    }
    std::cout << "\n";
}

void Compare(const Settings& cfg) {
    struct Row { const char* name; Runner run; };
    const Row rows[] = {
        { "BFS", RunBFS },
        { "DFS", RunDFS },
        { "DFS без повторов", RunDFSTree },
        { "IDS", RunIDS },
    };

    std::cout << "\n===== Сравнение алгоритмов: " << StateCode(cfg.start) << " -> " << StateCode(cfg.goal)
              << " =====\n"
              << "(DFS без контроля повторов - с ограничением глубины " << cfg.dfsTreeDepthLimit << ")\n\n"
              << "  Алгоритм         | Длина | Раскрыто | Порождено | max OPEN | CLOSED | Время, мкс\n"
              << "  -----------------+-------+----------+-----------+----------+--------+-----------\n";
    for (const Row& row : rows) {
        const SearchResult r = RunTimed(row.run, cfg, false);
        std::string name = row.name;
        // Выравнивание по ширине с учётом того, что кириллица в UTF-8 занимает 2 байта.
        size_t visible = 0;
        for (unsigned char ch : name) visible += (ch & 0xC0) != 0x80;
        name += std::string(visible < 16 ? 16 - visible : 0, ' ');

        std::cout << "  " << name << " | " << std::setw(5) << (r.found ? std::to_string(r.moves.size()) : "-")
                  << " | " << std::setw(8) << r.stats.expanded << " | " << std::setw(9) << r.stats.generated
                  << " | " << std::setw(8) << r.stats.maxFrontier << " | " << std::setw(6) << r.stats.closedSize
                  << " | " << std::setw(10) << std::fixed << std::setprecision(3) << r.stats.timeMicroseconds
                  << "\n";
        std::cout.unsetf(std::ios::fixed);
    }
    std::cout << "\n";
}

void ConfigureStates(Settings& cfg) {
    PrintLegend();
    std::cout << "Состояние вводится 4 символами 0/1 или L/R в порядке f w g c (пример: 0000, LLLL, 1010).\n";

    State s;
    std::string line = ReadLine("Начальное состояние [" + StateCode(cfg.start) + "]: ");
    if (!line.empty()) {
        if (!ParseState(line, s)) {
            std::cout << "Ошибка формата, состояние не изменено.\n";
        } else if (!IsValid(s)) {
            std::cout << "Состояние недопустимо (" << DangerReason(s) << "), не изменено.\n";
        } else {
            cfg.start = s;
        }
    }
    line = ReadLine("Целевое состояние [" + StateCode(cfg.goal) + "]: ");
    if (!line.empty()) {
        if (!ParseState(line, s)) {
            std::cout << "Ошибка формата, состояние не изменено.\n";
        } else if (!IsValid(s)) {
            std::cout << "Состояние недопустимо (" << DangerReason(s) << "), не изменено.\n";
        } else {
            cfg.goal = s;
        }
    }
    line = ReadLine("Ограничение глубины для DFS без контроля повторов [" +
                    std::to_string(cfg.dfsTreeDepthLimit) + "]: ");
    if (!line.empty()) {
        try {
            const int v = std::stoi(line);
            if (v >= 0 && v <= 30) cfg.dfsTreeDepthLimit = v;
            else std::cout << "Допустимо 0..30, значение не изменено.\n";
        } catch (...) {
            std::cout << "Ошибка формата, значение не изменено.\n";
        }
    }
    std::cout << "\n";
}

// Неинтерактивный режим: всё, что нужно для отчёта, одним прогоном.
void FullReport() {
    Settings cfg;
    PrintStateSpace();
    PrintAllSolutions(cfg);

    std::cout << "################ BFS ################\n";
    RunAndPrint(RunBFS, cfg);
    std::cout << "################ DFS ################\n";
    RunAndPrint(RunDFS, cfg);
    std::cout << "######## DFS без контроля повторов (L = " << cfg.dfsTreeDepthLimit << ") ########\n";
    RunAndPrint(RunDFSTree, cfg);
    std::cout << "################ IDS ################\n";
    RunAndPrint(RunIDS, cfg);

    Compare(cfg);

    // Старт на цикле графа состояний: кратчайший путь - 4 переправы,
    // но DFS первым пробует F+W и уходит по циклу длинной дорогой.
    Settings onCycle = cfg;
    ParseState("1110", onCycle.start);
    std::cout << "######## DFS из состояния " << StateCode(onCycle.start) << " ########\n";
    RunAndPrint(RunDFS, onCycle);
    Compare(onCycle);

    // Цель в одной переправе от старта.
    Settings nearGoal = cfg;
    ParseState("0101", nearGoal.start);
    Compare(nearGoal);
}

}  // namespace

int main(int argc, char* argv[]) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif

    std::cout << "=== Поиск в пространстве состояний: \"Волк, коза и капуста\" ===\n";

    if (argc > 1 && std::string(argv[1]) == "--report") {
        FullReport();
        return 0;
    }

    std::cout << "Крестьянину нужно перевезти через реку волка, козу и капусту. В лодку помещается\n"
                 "крестьянин и не более одного груза. Без крестьянина нельзя оставлять волка с козой\n"
                 "и козу с капустой.\n\n";

    Settings cfg;
    while (true) {
        std::cout << "Задача: [" << StateCode(cfg.start) << "] " << StateToString(cfg.start) << "  ->  ["
                  << StateCode(cfg.goal) << "] " << StateToString(cfg.goal) << "\n"
                  << "Трассировка: " << (cfg.trace ? "включена" : "выключена") << "\n\n"
                  << "  1 - BFS (поиск в ширину)\n"
                  << "  2 - DFS (поиск в глубину, со списком CLOSED)\n"
                  << "  3 - DFS без контроля повторов, ограничение глубины " << cfg.dfsTreeDepthLimit << "\n"
                  << "  4 - IDS (поиск с итеративным углублением)\n"
                  << "  5 - Сравнить все алгоритмы\n"
                  << "  6 - Показать пространство состояний\n"
                  << "  7 - Найти все решения (простые пути)\n"
                  << "  8 - Задать начальное / целевое состояние\n"
                  << "  9 - Включить / выключить трассировку\n"
                  << "  0 - Выход\n";
        const std::string choice = ReadLine("Ваш выбор: ");

        if (choice == "1") RunAndPrint(RunBFS, cfg);
        else if (choice == "2") RunAndPrint(RunDFS, cfg);
        else if (choice == "3") RunAndPrint(RunDFSTree, cfg);
        else if (choice == "4") RunAndPrint(RunIDS, cfg);
        else if (choice == "5") Compare(cfg);
        else if (choice == "6") PrintStateSpace();
        else if (choice == "7") PrintAllSolutions(cfg);
        else if (choice == "8") ConfigureStates(cfg);
        else if (choice == "9") { cfg.trace = !cfg.trace; std::cout << "\n"; }
        else if (choice == "0" || choice == "exit") break;
        else std::cout << "Неизвестная команда.\n\n";
    }
    return 0;
}
