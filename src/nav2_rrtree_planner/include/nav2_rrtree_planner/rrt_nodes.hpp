#include <vector>

struct coordsM
{
  int x, y;
};

struct coordsW
{
  double x, y;
};

class rrt_nodes
{
    public:
        int _xPos;
        int _yPos;
        rrt_nodes* _parent;
        std::vector<rrt_nodes*> _child;

        rrt_nodes()
        {
            _parent = nullptr;
        }

        rrt_nodes(int xPos, int yPos)
        {
            _xPos = xPos;
            _yPos = yPos;
            _parent = nullptr;

        }

        void setPos(int xPos, int yPos)
        {
            _xPos = xPos;
            _yPos = yPos;
        }

        rrt_nodes* getChild(int i)
        {
            return _child.at(i);
        }
};