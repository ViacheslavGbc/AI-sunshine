#include "rlImGui.h"
#include "Math.h"
#include <array>
#include <vector>
#include <queue>
#define SCREEN_WIDTH 1280
#define SCREEN_HEIGHT 720
#define TILE_COUNT 10

using namespace std;

constexpr float TILE_WIDTH = SCREEN_WIDTH / (float)TILE_COUNT;
constexpr float TILE_HEIGHT = SCREEN_HEIGHT / (float)TILE_COUNT;

using Map = array<array<size_t, TILE_COUNT>, TILE_COUNT>;

enum TileType : size_t
{
    MOUNTAIN,
    MUD,
    WATER,
    GRASS,
    AIR,
    COUNT
};

struct Cell
{
    int col = -1;
    int row = -1;
};

float Euclidean(Cell a, Cell b)
{
    return sqrtf(powf(b.col - a.col, 2.0f) + powf(b.row - a.row, 2.0f));
}

Cell ScreenToTile(Vector2 position)
{
    return { int(position.x / TILE_WIDTH), int(position.y / TILE_HEIGHT) };
}

Vector2 TileToScreen(Cell cell)
{
    return { cell.col * TILE_WIDTH, cell.row * TILE_HEIGHT };
}

Vector2 TileCenter(Cell cell)
{
    return TileToScreen(cell) + Vector2{ TILE_WIDTH * 0.5f, TILE_HEIGHT * 0.5f };
}

size_t Index(Cell cell)
{
    return cell.row * TILE_COUNT + cell.col;
}

float Cost(TileType type)
{
    static array<float, COUNT> costs
    {
        100.0f, // MOUNTAIN
        50.0f,  // MUD
        25.0f,  // WATER
        10.0f,  // GRASS
        0.0f,   // AIR
    };

    return costs[type];
}

vector<Cell> Neighbours(Cell cell)
{
    vector<Cell> neighbours;
    for (int row = -1; row <= 1; row++)
    {
        for (int col = -1; col <= 1; col++)
        {
            // Don't add the passed-in cell to the list
            if (row == cell.row && col == cell.col) continue;

            Cell neighbour{ cell.col + col, cell.row + row };
            if (neighbour.col >= 0 && neighbour.col < TILE_COUNT &&
                neighbour.row >= 0 && neighbour.row < TILE_COUNT)
                neighbours.push_back(neighbour);
        }
    }
    return neighbours;
}


struct Node
{
    Node()
    {
        Init();
    }

    Node(Cell cell)
    {
        Init(cell);
    }

    Node(Cell cell, float g, float h)
    {
        Init(cell, {}, g, h);
    }

    Node(Cell cell, Cell parent, float g, float h)
    {
        Init(cell, parent, g, h);
    }

    void Init(Cell cell = {}, Cell parent = {}, float g = 0.0f, float h = 0.0f)
    {
        this->cell = cell;
        this->parent = parent;
        this->g = g;
        this->h = h;
    }

    float F() { return g + h; }

    float g;
    float h;

    Cell cell;
    Cell parent;
};

bool operator==(Cell a, Cell b)
{
    return a.row == b.row && a.col == b.col;
}

bool Compare(Node a, Node b)
{
    return a.F() > b.F();
}

vector<Cell> FindPath(Cell start, Cell end, Map map, bool manhattan)
{
    // 1:1 mapping of graph nodes to tile map
    const int nodeCount = TILE_COUNT * TILE_COUNT;
    vector<Node> tileNodes(nodeCount);
    vector<bool> closedList(nodeCount, false);
    priority_queue<Node, vector<Node>, decltype(&Compare)> openList(Compare);
    tileNodes[Index(start)].parent = start;
    openList.push(start);

    // Loop until we've reached the goal, or explored every tile
    while (!openList.empty())
    {
        const Cell currentCell = openList.top().cell;

        // Stop exploring once we've found the goal
        if (currentCell == end)
            break;

        // Otherwise, add current cell to closed list and update g & h values of its neighbours
        openList.pop();
        closedList[Index(currentCell)] = true;

        float gNew, hNew;
        for (const Cell& neighbour : Neighbours(currentCell))
        {
            const size_t neighbourIndex = Index(neighbour);

            // Skip if already explored
            if (closedList[neighbourIndex]) continue;

            // Calculate scores
            gNew = Euclidean(currentCell, neighbour);   // Distance from current to adjacent
            hNew = Euclidean(neighbour, end);                   // Distance from adjacent to goal
            hNew += Cost((TileType)map[neighbour.row][neighbour.col]);

            // Append if unvisited or best score
            if (tileNodes[neighbourIndex].F() <= FLT_EPSILON /*unexplored*/ ||
                gNew + hNew < tileNodes[neighbourIndex].F() /*better score*/)
            {
                openList.push({ neighbour, gNew, hNew });
                tileNodes[neighbourIndex] = { neighbour, currentCell, gNew, hNew };
            }
        }
    }

    vector<Cell> path;
    Cell currentCell = end;
    size_t currentIndex = Index(currentCell);

    while (!(tileNodes[currentIndex].parent == currentCell))
    {
        path.push_back(currentCell);
        currentCell = tileNodes[currentIndex].parent;
        currentIndex = Index(currentCell);
    }
    path.push_back(start);
    reverse(path.begin(), path.end());

    return path;
}


void DrawTile(Cell cell, Color color)
{
    DrawRectangle(cell.col * TILE_WIDTH, cell.row * TILE_HEIGHT, TILE_WIDTH, TILE_HEIGHT, color);
}

void DrawTile(Cell cell, TileType type)
{
    Color color = WHITE;
    switch (type)
    {
    case MOUNTAIN:
        color = DARKGRAY;
        break;

    case MUD:
        color = BROWN;
        break;

    case WATER:
        color = BLUE;
        color.b = 180;
        break;

    case GRASS:
        color = GREEN;
        color.g = 180;
        break;
    }
    DrawTile(cell, color);
}

void DrawTile(Cell cell, Map map)
{
    DrawTile(cell, (TileType)map[cell.row][cell.col]);
}

struct Tile
{
    // Store neighbours (4 directions + diagonals)
    array<Tile*, 8> neighbours;
    float g;
    float h;
};

int main(void)
{
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Sunshine");
    SetTargetFPS(60);

    Map map
    {
        array<size_t, TILE_COUNT>{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        array<size_t, TILE_COUNT>{ 0, 3, 3, 1, 1, 1, 4, 4, 4, 0 },
        array<size_t, TILE_COUNT>{ 0, 3, 3, 1, 2, 1, 4, 4, 4, 0 },
        array<size_t, TILE_COUNT>{ 0, 3, 3, 1, 2, 1, 4, 4, 4, 0 },
        array<size_t, TILE_COUNT>{ 0, 3, 3, 1, 2, 1, 4, 4, 4, 0 },
        array<size_t, TILE_COUNT>{ 0, 3, 3, 1, 2, 1, 4, 4, 4, 0 },
        array<size_t, TILE_COUNT>{ 0, 3, 3, 1, 2, 1, 4, 4, 4, 0 },
        array<size_t, TILE_COUNT>{ 0, 3, 3, 1, 2, 1, 4, 4, 4, 0 },
        array<size_t, TILE_COUNT>{ 0, 3, 3, 1, 1, 1, 4, 4, 4, 0 },
        array<size_t, TILE_COUNT>{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    };

    Cell start{ 1, 1 };
    Cell goal{ 8, 8 };

    bool manhattan = true;
    vector<Cell> path = FindPath(start, goal, map, manhattan);

    while (!WindowShouldClose())
    {
        BeginDrawing();
        ClearBackground(RAYWHITE);
        
        for (int row = 0; row < TILE_COUNT; row++)
        {
            for (int col = 0; col < TILE_COUNT; col++)
            {
                Cell cell{ col, row };
                float g = Euclidean(cell, goal);
                float h = Euclidean(cell, goal);
                // TODO -- calculate cost of each tile to the goal tile
                DrawTile(cell, map);
                Vector2 texPos = TileCenter(cell);
                DrawText(TextFormat("F: %f", g + h), texPos.x, texPos.y, 10, MAROON);
            }
        }

        Vector2 cursor = GetMousePosition();
        Cell cursorTile = ScreenToTile(cursor);

        for (const Cell& cell : path)
            DrawTile(cell, RED);

        path = FindPath(start, cursorTile, map, manhattan);

        DrawTile(ScreenToTile(GetMousePosition()), RED);
        DrawTile(start, DARKBLUE);
        DrawTile(goal, SKYBLUE);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}