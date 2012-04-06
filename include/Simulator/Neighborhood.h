//
// Filename:        "Neighborhood.h"
//
// Programmer:      Ross Mead
// Last modified:   30Nov2009
//
// Description:     This class describes a robot cell neighborhood.
//

// preprocessor directives
#ifndef NEIGHBORHOOD_H
#define NEIGHBORHOOD_H
#include <time.h>
#include <vector>
#include <Simulator/Neighbor.h>
using namespace std;



// neighborhood size definitions
#define MAX_NEIGHBORHOOD_SIZE (10)



// forward declaration of a robot cell environment
class Environment;



// describes a robot cell neighborhood
class Neighborhood: public vector<Neighbor>
{

    public:

        // <constructors>
        Neighborhood();
        Neighborhood(const Neighborhood &nh);
        Neighborhood(const vector<Relationship> &r);
        Neighborhood(const vector<State> &s);

        // <destructors>
        ~Neighborhood();

        // <public mutator functions>
        bool addNbr(const Neighbor n);
        bool addNbr(const Relationship r = Relationship(),
                    const State        s = State());
        bool addNbr(const int  id,
                    const State  s       = State(),
                    const Vector desired = Vector(),
                    const Vector actual  = Vector());
        bool removeNbr(const Neighbor n);
        bool removeNbr(const int id);

        // <virtual public accessor functions>
        virtual vector<Relationship> getRelationships();
        virtual vector<State>        getStates();

        // <public accessor functions>
        Neighbor*        getNbr(const int pos);
        vector<Neighbor> getNbrs()  const;
        int            getNNbrs() const;

        // <public utility functions>
        bool updateNbr(Neighbor &n, const State &s);
        bool updateNbr(const int id, const State &s);
        bool isNbr(const Neighbor n);
        bool isNbr(const int id);
        void clearNbrs();

        // <public single neighbor member functions>
        Neighbor* anyNbr();
        Neighbor* firstNbr();
        Neighbor* secondNbr();
        Neighbor* lastNbr();
        Neighbor* closestNbr(const Vector c = Vector());
        Neighbor* furthestNbr(const Vector c = Vector());

        // <public single neighbor property member functions>
        Neighbor* nbrWithID(const int id);
        Neighbor* nbrWithGradient(const Vector grad);
        Neighbor* nbrWithMinGradient(const Vector c = Vector());
        Neighbor* nbrWithMaxGradient(const Vector c = Vector());
        Neighbor* nbrWithMinStep();
        Neighbor* nbrWithMaxStep();

        // <public neighbor list member functions>
        void sortByID();
        void sortByGradient(const Vector c = Vector());
        void sortByDistance(const Vector c = Vector());
        void sortByAngle(const Vector c = Vector());
        void sortByAbsAngle(const Vector c = Vector());
        bool swapNbrs(const int i, const int j);
};  // Neighborhood

#endif
