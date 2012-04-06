//
// Filename:        "Neighborhood.cpp"
//
// Programmer:      Ross Mead
// Last modified:   30Nov2009
//
// Description:     This class implements a robot cell neighborhood.
//

// preprocessor directives
#include <Simulator/Neighborhood.h>
#include <ctime>
#include <stdio.h>


// <constructors>

//
// Neighborhood()
// Last modified: 30Nov2009
//
// Default constructor that initializes
// this neighborhood to the appropriate values.
//
// Returns:     <none>
// Parameters:  <none>
//
Neighborhood::Neighborhood(): vector<Neighbor>()
{
    reserve(MAX_NEIGHBORHOOD_SIZE);
}   // Neighborhood()



//
// Neighborhood(nh)
// Last modified: 30Nov2009
//
// Copy constructor that copies the contents of the
// parameterized neighborhood into this neighborhood.
//
// Returns:     <none>
// Parameters:
//      nh      in/out      the neighborhood being copied
//
Neighborhood::Neighborhood(const Neighborhood &nh)
{
    reserve(MAX_NEIGHBORHOOD_SIZE);
    *this = nh;
}   // Neighborhood(const Neighborhood &)



//
// Neighborhood(r)
// Last modified: 29Nov2009
//
// Copy constructor that copies the contents of the
// parameterized list of relationships into this neighborhood.
//
// Returns:     <none>
// Parameters:
//      r       in/out      the list of relationships being copied
//
Neighborhood::Neighborhood(const vector<Relationship> &r)
{
    for (int i = 0; i < r.size(); ++i) addNbr(r[i], State());
}   // Neighborhood(const vector<Relationship> &)



//
// Neighborhood(s)
// Last modified: 29Nov2009
//
// Copy constructor that copies the contents of the
// parameterized list of states into this neighborhood.
//
// Returns:     <none>
// Parameters:
//      s       in/out      the list of states being copied
//
Neighborhood::Neighborhood(const vector<State> &s)
{
    for (int i = 0; i < s.size(); ++i) addNbr(Relationship(), s[i]);
}   // Neighborhood(const vector<State> &)



// <destructors>

//
// ~Neighborhood()
// Last modified: 02Sep2006
//
// Destructor that clears this neighborhood.
//
// Returns:     <none>
// Parameters:  <none>
//
Neighborhood::~Neighborhood()
{
}   // ~Neighborhood()



// <public mutator functions>

//
// bool addNbr(n)
// Last modified: 30Nov2009
//
// Attempts to add the parameterized neighbor to the neighborhood,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      n       in      the neighbor being added
//
bool Neighborhood::addNbr(const Neighbor n)
{
    if (size() < MAX_NEIGHBORHOOD_SIZE)
    {
        push_back(n);
        return true;
    }
    return false;
}   // addNbr(const Neighbor)



//
// bool addNbr(r, s)
// Last modified: 02Sep2006
//
// Attempts to add a neighbor to the neighborhood
// based on the parameterized relationship and state,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      r       in      the relationship of the neighbor being added
//      s       in      the state of the neighbor being added
//
bool Neighborhood::addNbr(const Relationship r, const State s)
{
    return addNbr(Neighbor(r, s));
}   // addNbr(const Relationship, const State)



//
// bool addNbr(id, s, desired, actual)
// Last modified: 02Sep2006
//
// Attempts to add a neighbor to the neighborhood
// based on the parameterized ID, state, and relationships,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      id          in  the ID of the neighbor being added
//      s           in  the state of the neighbor being added
//      desired     in  the desired relationship of the neighbor being added
//      actual      in  the actual relationship of the neighbor being added
//
bool Neighborhood::addNbr(const int  id,      const State  s,
                          const Vector desired, const Vector actual)
{
    return addNbr(Neighbor(id, s, desired, actual));
}   // addNbr(const int, const State, const Vector, const Vector)



//
// bool removeNbr(n)
// Last modified: 02Sep2006
//
// Attempts to remove the parameterized neighbor,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      n       in      the neighbor to remove
//
bool Neighborhood::removeNbr(const Neighbor n)
{
    return removeNbr(n.ID);
}   // removeNbr(const Neighbor)



//
// bool removeNbr(id)
// Last modified: 30Nov2009
//
// Attempts to remove the neighbor with the parameterized ID,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      id      in      the ID of the neighbor to remove
//
bool Neighborhood::removeNbr(const int id)
{
    Neighbor nbr;
    for (int i = 0; i < size(); ++i)
    {
        nbr = at(i);
        if (nbr.ID == id)
        {
            erase(begin() + i);
            return true;
        }
    }
    return false;
}   // removeNbr(const int)



// <virtual public accessor functions>

//
// vector<Relationship> getRelationships()
// Last modified: 30Nov2009
//
// Returns the list of neighbor relationships.
//
// Returns:     list of neighbor relationships
// Parameters:  <none>
//
vector<Relationship> Neighborhood::getRelationships()
{
    vector<Relationship> rels;
    Neighbor currNbr;
    for (int i = 0; i < size(); ++i)
    {
        currNbr = at(i);
        rels.push_back(currNbr);
    }
    return rels;
}   // getRelationships()



//
// vector<State> getStates()
// Last modified: 30Nov2009
//
// Returns the list of neighbor states.
//
// Returns:     list of neighbor states
// Parameters:  <none>
//
vector<State> Neighborhood::getStates()
{
    vector<State> states;
    Neighbor currNbr;
    for (int i = 0; i < size(); ++i)
    {
        currNbr = at(i);
        states.push_back(currNbr);
    }
    return states;
}   // getStates()



// <public accessor functions>

//
// Neighbor* getNbr(pos)
// Last modified: 30Nov2009
//
// Returns the neighbor at the parameterized position in this neighborhood.
//
// Returns:     the neighbor at the parameterized position in this neighborhood
// Parameters:
//      pos     in      the position of the desired neighbor (default 0)
//
Neighbor* Neighborhood::getNbr(const int pos)
{
    if ((pos < 0) || (pos >= size())) return NULL;
    return &at(pos);
}   // getNbr(const int) const



//
// vector<Neighbor> getNbrs() const
// Last modified: 29Nov2009
//
// Returns the list of neighbors in this neighborhood.
//
// Returns:     list of neighbors in this neighborhood
// Parameters:  <none>
//
vector<Neighbor> Neighborhood::getNbrs() const
{
    return *this;
}   // getNbrs() const



//
// int getNNbrs() const
// Last modified: 29Nov2009
//
// Returns the number of neighbors in this neighborhood.
//
// Returns:     the number of neighbors in this neighborhood
// Parameters:  <none>
//
int Neighborhood::getNNbrs() const
{
    return size();
}   // getNNbrs() const



// <public utility functions>

//
// bool updateNbr(n, s)
// Last modified: 07Sep2006
//
// Updates the state of the parameterized neighbor,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      n       in/out  the neighbor being updated
//      s       in/out  the updated state of the neighbor
//
bool Neighborhood::updateNbr(Neighbor &n, const State &s)
{
    n = s;
    return true;
}   // updateNbr(Neighbor* &, const State &)



//
// bool updateNbr(id, s)
// Last modified: 07Sep2006
//
// Updates the state of the neighbor with the parameterized ID,
// returning true if successful, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      id      in      the ID of the neighbor being updated
//      s       in/out  the updated state of the neighbor
//
bool Neighborhood::updateNbr(const int id, const State &s)
{
    Neighbor *nbr = nbrWithID(id);
    if (nbr == NULL) return false;
    return updateNbr(*nbr, s);
}   // updateNbr(const int, const State &)



//
// bool isNbr(n)
// Last modified: 02Sep2006
//
// Returns true if the parameterized neighbor
// is in this neighborhood, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      n       in      the neighbor to find
//
bool Neighborhood::isNbr(const Neighbor n)
{
    return isNbr(n.ID);
}   // isNbr(const Neighbor)



//
// bool isNbr(id)
// Last modified: 30Nov2009
//
// Returns true if the neighbor with the parameterized ID
// is in this neighborhood, false otherwise.
//
// Returns:     true if successful, false otherwise
// Parameters:
//      id      in      the ID of the neighbor to find
//
bool Neighborhood::isNbr(const int id)
{
    Neighbor nbr;
    for (int i = 0; i < size(); ++i)
    {
        nbr = at(i);
        if (nbr.ID == id) return true;
    }
    return false;
}   // isNbr(const int)

//
// void clearNbrs()
// Last modified: 02Sep2006
//
// Clears the neighbors in this neighborhood.
//
// Returns:     <none>
// Parameters:  <none>
//
void Neighborhood::clearNbrs()
{
    clear();
}   // clearNbrs()



// <public single neighbor member functions>

//
// Neighbor* anyNbr()
// Last modified: 30Nov2009
//
// Returns a random neighbor in this neighborhood.
//
// Returns:     random neighbor in this neighborhood
// Parameters:  <none>
//
Neighbor* Neighborhood::anyNbr()
{
    srand(time(NULL));
    if (!empty()) return &at(irand(0, size()));
    return NULL;
}   // anyNbr()



//
// Neighbor* firstNbr()
// Last modified: 30Nov2009
//
// Returns the first neighbor in this neighborhood.
//
// Returns:     the first neighbor in this neighborhood
// Parameters:  <none>
//
Neighbor* Neighborhood::firstNbr()
{
    if (empty()) return NULL;
    return &at(0);
}   // firstNbr(Neighborhood &nh)



//
// Neighbor* secondNbr()
// Last modified: 30Nov2009
//
// Returns the second neighbor in this neighborhood.
//
// Returns:     the second neighbor in this neighborhood
// Parameters:  <none>
//
Neighbor* Neighborhood::secondNbr()
{
    if (size() < 2) return NULL;
    return &at(1);
}   // secondNbr()



//
// Neighbor* lastNbr()
// Last modified: 30Nov2009
//
// Returns the last neighbor in this neighborhood.
//
// Returns:     the last neighbor in this neighborhood
// Parameters:  <none>
//
Neighbor* Neighborhood::lastNbr()
{
    if (empty()) return NULL;
    return &at(size() - 1);
}   // lastNbr()



//
// Neighbor* closestNbr(v)
// Last modified: 30Nov2009
//
// Returns the closest neighbor in this neighborhood
// as determined by the parameterized difference vector.
//
// Returns:     the closest neighbor in this neighborhood
// Parameters:
//      v       in      the difference vector
//
Neighbor* Neighborhood::closestNbr(const Vector v)
{
    Neighbor currNbr;
    float  minDist  = 0.0f, currDist = 0.0f;
    int    minIndex = ID_NO_NBR;
    for (int i = 0; i < size(); ++i)
    {
        currNbr = at(i);
        if ((currNbr.ID != ID_NO_NBR) &&
            (((currDist = (currNbr.relActual - v).magnitude()) < minDist) ||
             (minIndex == ID_NO_NBR)))
        {
            minDist   = currDist;
            minIndex  = i;
        }
    }
    return ((minIndex < 0) || (minIndex > size()))
            ? NULL : &at(minIndex);
}   // closestNbr(const Vector)



//
// Neighbor* furthestNbr(v)
// Last modified: 30Nov2009
//
// Returns the furthest neighbor in this neighborhood
// as determined by the parameterized difference vector.
//
// Returns:     the furthest neighbor in this neighborhood
// Parameters:
//      v       in      the difference vector
//
Neighbor* Neighborhood::furthestNbr(const Vector v)
{
    Neighbor currNbr;
    float  maxDist  = 0.0f, currDist = 0.0f;
    int    maxIndex = ID_NO_NBR;
    for (int i = 0; i < size(); ++i)
    {
        currNbr = at(i);
        if ((currNbr.ID != ID_NO_NBR) &&
            (((currDist = (currNbr.relActual - v).magnitude()) > maxDist) ||
             (maxIndex == ID_NO_NBR)))
        {
            maxDist   = currDist;
            maxIndex  = i;
        }
    }
    return ((maxIndex < 0) || (maxIndex > size()))
            ? NULL : &at(maxIndex);
}   // furthestNbr(const Vector)



// <public single neighbor property member functions>

//
// Neighbor* nbrWithID(id)
// Last modified: 30Nov2009
//
// Returns the neighbor (in this neighborhood) with the parameterized ID.
//
// Returns:     the neighbor with the parameterized ID
// Parameters:
//      id      in      the ID of the neighbor to find
//
Neighbor* Neighborhood::nbrWithID(const int id)
{
    Neighbor currNbr;
    for (int i = 0; i < size(); ++i)
    {
        currNbr = at(i);
        if (currNbr.ID == id) return &at(i);
    }
    return NULL;
}   // nbrWithID(const int)



//
// Neighbor* nbrWithGradient(grad)
// Last modified: 30Nov2009
//
// Returns the neighbor (in this neighborhood) with the parameterized gradient.
//
// Returns:     the neighbor with the parameterized gradient
// Parameters:
//      gradient    in  the gradient of the neighbor to find
//
Neighbor* Neighborhood::nbrWithGradient(const Vector grad)
{
    Neighbor currNbr;
    for (int i = 0; i < size(); ++i)
    {
        currNbr = at(i);
        if (currNbr.gradient == grad) return &at(i);
    }
    return NULL;
}   // nbrWithGradient(const Vector)



//
// Neighbor* nbrWithMinGradient(v)
// Last modified: 30Nov2009
//
// Returns the neighbor (in this neighborhood) with the minimum gradient
// distance as determined by the parameterized difference vector.
//
// Returns:     the neighbor with the minimum gradient
// Parameters:
//      v       in      the difference vector
//
Neighbor* Neighborhood::nbrWithMinGradient(const Vector v)
{
    Neighbor currNbr;
    float  minGrad  = 0.0f, currGrad = 0.0f;
    int    minIndex = ID_NO_NBR;
    for (int i = 0; i < size(); ++i)
    {
        currNbr = at(i);
        if ((currNbr.ID != ID_NO_NBR) &&
            (((currGrad = (currNbr.gradient - v).magnitude())
              < minGrad) || (minIndex == ID_NO_NBR)))
        {
            minGrad   = currGrad;
            minIndex  = i;
        }
    }
    return ((minIndex < 0) || (minIndex > size()))
            ? NULL : &at(minIndex);
}   // nbrWithMinGradient(const Vector)



//
// Neighbor* nbrWithMaxGradient(v)
// Last modified: 30Nov2009
//
// Returns the neighbor (in this neighborhood) with the maximum gradient
// distance as determined by the parameterized difference vector.
//
// Returns:     the neighbor with the maximum gradient
// Parameters:
//      v       in      the difference vector
//
Neighbor* Neighborhood::nbrWithMaxGradient(const Vector v)
{
    Neighbor currNbr;
    float  maxGrad  = 0.0f, currGrad = 0.0f;
    int    maxIndex = ID_NO_NBR;
    for (int i = 0; i < size(); ++i)
    {
        currNbr = at(i);
        if ((currNbr.ID != ID_NO_NBR) &&
            (((currGrad = (currNbr.gradient - v).magnitude())
              > maxGrad) || (maxIndex == ID_NO_NBR)))
        {
            maxGrad   = currGrad;
            maxIndex  = i;
        }
    }
    return ((maxIndex < 0) || (maxIndex > size()))
            ? NULL : &at(maxIndex);
}   // nbrWithMaxGradient(const Vector)



//
// Neighbor* nbrWithMinStep()
// Last modified: 30Nov2009
//
// Returns the neighbor (in this neighborhood) with the minimum step.
//
// Returns:    the neighbor with the minimum step
// Parameters: <none>
//
Neighbor* Neighborhood::nbrWithMinStep()
{
    Neighbor currNbr;
    int    minStep  = 0, currStep = 0;
    int    minIndex = ID_NO_NBR;
    for (int i = 0; i < size(); ++i)
    {
        currNbr = at(i);
        if ((currNbr.ID != ID_NO_NBR) &&
            (((currStep = currNbr.tStep) < minStep) ||
             (minIndex == ID_NO_NBR)))
        {
            minStep   = currStep;
            minIndex  = i;
        }
    }
    return ((minIndex < 0) || (minIndex > size()))
            ? NULL : &at(minIndex);
}   // nbrWithMinStep()



//
// Neighbor* nbrWithMaxStep()
// Last modified: 30Nov2009
//
// Returns the neighbor (in this neighborhood) with the maximum step.
//
// Returns:    the neighbor with the maximum step
// Parameters: <none>
//
Neighbor* Neighborhood::nbrWithMaxStep()
{
    Neighbor currNbr;
    int    maxStep  = 0, currStep = 0;
    int    maxIndex = ID_NO_NBR;
    for (int i = 0; i < size(); ++i)
    {
        currNbr = at(i);
        if ((currNbr.ID != ID_NO_NBR) &&
            (((currStep = currNbr.tStep) > maxStep) ||
             (maxIndex == ID_NO_NBR)))
        {
            maxStep   = currStep;
            maxIndex  = i;
        }
    }
    return ((maxIndex < 0) || (maxIndex > size()))
            ? NULL : &at(maxIndex);
}   // nbrWithMaxStep()



// <public neighbor list member functions>

//
// void sortByID()
// Last modified: 29Nov2009
//
// Sorts the neighborhood based upon neighbor ID.
//
// Returns:     <none>
// Parameters:  <none>
//
void Neighborhood::sortByID()
{
    for (int i = 0; i < size() - 1; ++i)
        for (int j = i; j < size(); ++j)
            if (getNbr(i)->ID > getNbr(j)->ID) swapNbrs(i, j);
}   // sortByID()



//
// void sortByGradient(v)
// Last modified: 29Nov2009
//
// Sorts the neighborhood based upon neighbor gradient
// as determined by the parameterized difference vector.
//
// Returns:     <none>
// Parameters:
//      v       in      the difference vector
//
void Neighborhood::sortByGradient(const Vector v)
{
    for (int i = 0; i < size() - 1; ++i)
        for (int j = i; j < size(); ++j)
            if ((getNbr(i)->gradient - v).magnitude() >
                (getNbr(j)->gradient - v).magnitude()) swapNbrs(i, j);
}   // sortByGradient(const Vector)



//
// void sortByDistance(v)
// Last modified: 29Nov2009
//
// Sorts the neighborhood based upon neighbor distance
// as determined by the parameterized difference vector.
//
// Returns:     <none>
// Parameters:
//      v       in      the difference vector
//
void Neighborhood::sortByDistance(const Vector v)
{
    for (int i = 0; i < size() - 1; ++i)
        for (int j = i; j < size(); ++j)
            if ((getNbr(i)->relActual - v).magnitude() >
                (getNbr(j)->relActual - v).magnitude()) swapNbrs(i, j);
}   // sortByDistance(const Vector)



//
// void sortByAngle(v)
// Last modified: 29Nov2009
//
// Sorts the neighborhood based upon the angle of the
// neighbor-relative and the parameterized difference vector.
//
// Returns:     <none>
// Parameters:
//      v       in      the difference vector
//
void Neighborhood::sortByAngle(const Vector v)
{
    for (int i = 0; i < size() - 1; ++i)
        for (int j = i; j < size(); ++j)
            if ((getNbr(i)->relActual - v).angle() >
                (getNbr(j)->relActual - v).angle()) swapNbrs(i, j);
}   // sortByAngle(const Vector)



//
// void sortByAbsAngle(v)
// Last modified: 29Nov2009
//
// Sorts the neighborhood based upon the absolute value of the angle
// of the neighbor-relative and the parameterized difference vector.
//
// Returns:     <none>
// Parameters:
//      v       in      the difference vector
//
void Neighborhood::sortByAbsAngle(const Vector v)
{
    for (int i = 0; i < size() - 1; ++i)
        for (int j = i; j < size(); ++j)
            if (abs((getNbr(i)->relActual - v).angle()) >
                abs((getNbr(j)->relActual - v).angle())) swapNbrs(i, j);
}   // sortByAbsAngle(const Vector)



//
// bool swapNbrs(i, j)
// Last modified: 30Nov2009
//
// Swaps the neighbors at the parameterized indeces.
//
// Returns:     <none>
// Parameters:
//      i       in      the index of the first neigbor
//      j       in      the index of the second neigbor
//
bool Neighborhood::swapNbrs(const int i, const int j)
{
    if ((i < 0) || (i >= size()) || (j < 0) || (j >= size())) return false;
    Neighbor tempNbr = at(i);
    at(i)            = at(j);
    at(j)            = tempNbr;
    return true;
}   // swapNbrs(const int, const int)
