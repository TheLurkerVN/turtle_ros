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
    private:
        coordsM node_coords;
        //int _xPos;
        //int _yPos;
        rrt_nodes* _parent;
        std::vector<rrt_nodes*> _child;

    public:
        rrt_nodes()
        {
            _parent = nullptr;
        }

        rrt_nodes(int xPos, int yPos)
        {
            node_coords.x = xPos;
            node_coords.y = yPos;
            _parent = nullptr;

        }

        int getPosX()
        {
            return node_coords.x;
        }

        int getPosY()
        {
            return node_coords.y;
        }

        void setPos(int xPos, int yPos)
        {
            node_coords.x = xPos;
            node_coords.y = yPos;
        }

        rrt_nodes* getParent()
        {
            return _parent;
        }

        void setParent(rrt_nodes* parent)
        {
            _parent = parent;
        }

        std::vector<rrt_nodes*> getChildArr()
        {
            return _child;
        }

        int getChildSize()
        {
            return _child.size();
        }

        rrt_nodes* getChild(int i)
        {
            return _child.at(i);
        }


        void setChild(rrt_nodes* child)
        {
            _child.push_back(child);
        }
};