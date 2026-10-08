#include "Solver.h"

#include <algorithm>
#include <array>
#include <deque>
#include <iomanip>
#include <sstream>

namespace {

// Узел дерева поиска. Дерево хранится в векторе, связь с родителем - по индексу.
struct Node {
    State state = 0;
    int parent = -1;
    Move move;     // оператор, которым узел получен из родителя
    int depth = 0;
};

void BuildPath(const std::vector<Node>& tree, int idx, SearchResult& r) {
    r.path.clear();
    r.moves.clear();
    for (int i = idx; i != -1; i = tree[i].parent) {
        r.path.push_back(tree[i].state);
        if (tree[i].parent != -1) r.moves.push_back(tree[i].move);
    }
    std::reverse(r.path.begin(), r.path.end());
    std::reverse(r.moves.begin(), r.moves.end());
}

std::string NodeText(State s) {
    return "[" + StateCode(s) + "] " + StateToString(s);
}

std::string TransitionText(const Transition& t) {
    std::ostringstream os;
    os << std::left << std::setw(4) << CargoShort(t.move.cargo) << "-> " << NodeText(t.to);
    return os.str();
}

template <typename Container, typename Getter>
std::string StateList(const Container& c, Getter get) {
    std::string s = "[";
    bool first = true;
    for (const auto& x : c) {
        if (!first) s += ", ";
        s += StateCode(get(x));
        first = false;
    }
    return s + "]";
}

std::string ClosedList(const std::array<bool, STATE_COUNT>& closed) {
    std::string s = "{";
    bool first = true;
    for (int i = 0; i < STATE_COUNT; ++i) {
        if (!closed[i]) continue;
        if (!first) s += ", ";
        s += StateCode(static_cast<State>(i));
        first = false;
    }
    return s + "}";
}

size_t CountTrue(const std::array<bool, STATE_COUNT>& a) {
    return static_cast<size_t>(std::count(a.begin(), a.end(), true));
}

}  // namespace

// ---------------------------------------------------------------- BFS ----

SearchResult SolveBFS(State start, State goal, std::ostream* trace) {
    SearchResult r;
    r.algorithm = "BFS";

    std::vector<Node> tree;
    tree.push_back(Node{ start, -1, Move{}, 0 });
    r.stats.generated = 1;

    if (start == goal) {
        r.found = true;
        BuildPath(tree, 0, r);
        if (trace) *trace << "Начальное состояние совпадает с целевым.\n";
        return r;
    }

    std::deque<int> open{ 0 };                    // очередь FIFO
    std::array<bool, STATE_COUNT> reached{};      // состояние уже попадало в очередь
    reached[start] = true;
    r.stats.maxFrontier = 1;

    int step = 0;
    while (!open.empty()) {
        const int idx = open.front();
        open.pop_front();
        const Node node = tree[idx];  
        ++r.stats.expanded;
        r.stats.maxDepth = std::max(r.stats.maxDepth, node.depth);

        if (trace)
            *trace << "Шаг " << ++step << ". Из очереди извлечён узел #" << idx << " "
                   << NodeText(node.state) << ", глубина " << node.depth << "\n";

        for (const Transition& t : AllTransitions(node.state)) {
            if (!t.valid) {
                ++r.stats.rejectedUnsafe;
                if (trace) *trace << "    " << TransitionText(t) << "  отброшен: " << DangerReason(t.to) << "\n";
                continue;
            }
            if (reached[t.to]) {
                ++r.stats.rejectedRepeated;
                if (trace) *trace << "    " << TransitionText(t) << "  отброшен: уже встречалось\n";
                continue;
            }

            reached[t.to] = true;
            tree.push_back(Node{ t.to, idx, t.move, node.depth + 1 });
            ++r.stats.generated;
            const int child = static_cast<int>(tree.size()) - 1;
            if (trace) *trace << "    " << TransitionText(t) << "  новый узел #" << child << "\n";

            if (t.to == goal) {
                r.found = true;
                r.stats.maxDepth = std::max(r.stats.maxDepth, node.depth + 1);
                r.stats.closedSize = CountTrue(reached);
                BuildPath(tree, child, r);
                if (trace) *trace << "  Узел #" << child << " - целевой. Поиск завершён.\n";
                return r;
            }
            open.push_back(child);
        }

        r.stats.maxFrontier = std::max(r.stats.maxFrontier, open.size());
        if (trace)
            *trace << "  OPEN (очередь):   " << StateList(open, [&](int i) { return tree[i].state; }) << "\n"
                   << "  Встречено:        " << ClosedList(reached) << "\n";
    }

    r.stats.closedSize = CountTrue(reached);
    if (trace) *trace << "Очередь пуста - решения нет.\n";
    return r;
}

// ---------------------------------------------------------------- DFS ----

SearchResult SolveDFS(State start, State goal, std::ostream* trace) {
    SearchResult r;
    r.algorithm = "DFS";

    std::vector<Node> tree;
    tree.push_back(Node{ start, -1, Move{}, 0 });
    r.stats.generated = 1;

    std::vector<int> open{ 0 };                   // стек LIFO, вершина - back()
    std::array<bool, STATE_COUNT> closed{};       // раскрытые состояния
    r.stats.maxFrontier = 1;

    int step = 0;
    while (!open.empty()) {
        const int idx = open.back();
        open.pop_back();
        const Node node = tree[idx];

        if (closed[node.state]) {
            // Состояние было положено в стек дважды и уже раскрыто по другому пути.
            if (trace) *trace << "Узел #" << idx << " [" << StateCode(node.state) << "] уже раскрыт - пропускаем\n";
            continue;
        }

        if (trace)
            *trace << "Шаг " << ++step << ". Из стека извлечён узел #" << idx << " "
                   << NodeText(node.state) << ", глубина " << node.depth << "\n";

        if (node.state == goal) {
            r.found = true;
            r.stats.closedSize = CountTrue(closed);
            BuildPath(tree, idx, r);
            if (trace) *trace << "  Узел #" << idx << " - целевой. Поиск завершён.\n";
            return r;
        }
        ++r.stats.expanded;
        r.stats.maxDepth = std::max(r.stats.maxDepth, node.depth);
        closed[node.state] = true;

        std::vector<int> children;
        for (const Transition& t : AllTransitions(node.state)) {
            if (!t.valid) {
                ++r.stats.rejectedUnsafe;
                if (trace) *trace << "    " << TransitionText(t) << "  отброшен: " << DangerReason(t.to) << "\n";
                continue;
            }
            if (closed[t.to]) {
                ++r.stats.rejectedRepeated;
                if (trace) *trace << "    " << TransitionText(t) << "  отброшен: уже раскрыто\n";
                continue;
            }
            tree.push_back(Node{ t.to, idx, t.move, node.depth + 1 });
            ++r.stats.generated;
            children.push_back(static_cast<int>(tree.size()) - 1);
            if (trace) *trace << "    " << TransitionText(t) << "  новый узел #" << children.back() << "\n";
        }
        // Потомки кладутся в обратном порядке, чтобы первым раскрывался
        // первый по порядку генерации оператор.
        for (auto it = children.rbegin(); it != children.rend(); ++it) open.push_back(*it);

        r.stats.maxFrontier = std::max(r.stats.maxFrontier, open.size());
        if (trace)
            *trace << "  OPEN (стек, вершина справа): " << StateList(open, [&](int i) { return tree[i].state; }) << "\n"
                   << "  CLOSED:                      " << ClosedList(closed) << "\n";
    }

    r.stats.closedSize = CountTrue(closed);
    if (trace) *trace << "Стек пуст - решение не найдено.\n";
    return r;
}

// ---------------------------------------------------------------- IDS ----

namespace {

enum class DlsStatus { Found, Cutoff, Failure };

struct DlsContext {
    State goal = 0;
    int limit = 0;
    std::array<bool, STATE_COUNT> onPath{};
    std::vector<State> path;
    std::vector<Move> moves;
    SearchStats* stats = nullptr;
    IdsIteration* iteration = nullptr;
    std::ostream* trace = nullptr;
};

// Рекурсивный поиск в глубину с ограничением глубины (Depth-Limited Search).
// Cutoff - решение не найдено, но поиск был прерван ограничением глубины;
// Failure - решения нет вообще (на любой глубине).
DlsStatus DepthLimited(DlsContext& ctx, int depth) {
    const State s = ctx.path.back();
    const std::string indent(2 + 2 * depth, ' ');
    ctx.stats->maxFrontier = std::max(ctx.stats->maxFrontier, ctx.path.size());

    if (s == ctx.goal) {
        if (ctx.trace) *ctx.trace << indent << NodeText(s) << "  - ЦЕЛЬ\n";
        return DlsStatus::Found;
    }
    if (depth == ctx.limit) {
        if (ctx.trace) *ctx.trace << indent << NodeText(s) << "  - отсечение по глубине\n";
        return DlsStatus::Cutoff;
    }

    if (ctx.trace) *ctx.trace << indent << NodeText(s) << "\n";
    ++ctx.stats->expanded;
    ++ctx.iteration->expanded;
    ctx.stats->maxDepth = std::max(ctx.stats->maxDepth, depth);

    bool cutoff = false;
    for (const Transition& t : AllTransitions(s)) {
        if (!t.valid) {
            ++ctx.stats->rejectedUnsafe;
            continue;
        }
        if (ctx.onPath[t.to]) {  // цикл: состояние уже есть на текущем пути
            ++ctx.stats->rejectedRepeated;
            continue;
        }

        ++ctx.stats->generated;
        ++ctx.iteration->generated;
        ctx.onPath[t.to] = true;
        ctx.path.push_back(t.to);
        ctx.moves.push_back(t.move);

        const DlsStatus st = DepthLimited(ctx, depth + 1);
        if (st == DlsStatus::Found) return st;
        if (st == DlsStatus::Cutoff) cutoff = true;

        ctx.onPath[t.to] = false;
        ctx.path.pop_back();
        ctx.moves.pop_back();
    }
    return cutoff ? DlsStatus::Cutoff : DlsStatus::Failure;
}

}  // namespace

SearchResult SolveIDS(State start, State goal, int maxDepth, std::ostream* trace) {
    SearchResult r;
    r.algorithm = "IDS";

    for (int limit = 0; limit <= maxDepth; ++limit) {
        r.iterations.push_back(IdsIteration{ limit });

        DlsContext ctx;
        ctx.goal = goal;
        ctx.limit = limit;
        ctx.onPath[start] = true;
        ctx.path.push_back(start);
        ctx.stats = &r.stats;
        ctx.iteration = &r.iterations.back();
        ctx.trace = trace;

        ++r.stats.generated;  // корень порождается заново на каждой итерации
        ++r.iterations.back().generated;
        if (trace) *trace << "Итерация: ограничение глубины L = " << limit << "\n";

        const DlsStatus st = DepthLimited(ctx, 0);
        if (st == DlsStatus::Found) {
            r.found = true;
            r.iterations.back().found = true;
            r.path = ctx.path;
            r.moves = ctx.moves;
            if (trace) *trace << "Решение найдено на итерации L = " << limit << ".\n";
            return r;
        }
        if (st == DlsStatus::Failure) {
            // Ни одна ветвь не была отсечена глубиной - увеличивать L бессмысленно.
            if (trace) *trace << "Дерево исчерпано без отсечений - решения нет.\n";
            return r;
        }
    }
    if (trace) *trace << "Превышена максимальная глубина " << maxDepth << ".\n";
    return r;
}

// ------------------------------------------------- перебор всех решений ----

namespace {

void EnumerateRec(State goal, std::array<bool, STATE_COUNT>& onPath, SearchResult& cur,
                  std::vector<SearchResult>& out) {
    const State s = cur.path.back();
    if (s == goal) {
        out.push_back(cur);
        return;
    }
    for (const Transition& t : Successors(s)) {
        if (onPath[t.to]) continue;
        onPath[t.to] = true;
        cur.path.push_back(t.to);
        cur.moves.push_back(t.move);
        EnumerateRec(goal, onPath, cur, out);
        cur.path.pop_back();
        cur.moves.pop_back();
        onPath[t.to] = false;
    }
}

}  // namespace

std::vector<SearchResult> EnumerateAllSolutions(State start, State goal) {
    std::vector<SearchResult> out;
    SearchResult cur;
    cur.found = true;
    cur.path.push_back(start);
    std::array<bool, STATE_COUNT> onPath{};
    onPath[start] = true;
    EnumerateRec(goal, onPath, cur, out);
    std::sort(out.begin(), out.end(),
              [](const SearchResult& a, const SearchResult& b) { return a.moves.size() < b.moves.size(); });
    return out;
}
