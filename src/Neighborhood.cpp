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



// Default constructor that initializes
// this neighborhood to the appropriate values.
Neighborhood::Neighborhood(): vector<Neighbor>()
{
    reserve(MAX_NEIGHBORHOOD_SIZE);
}


// Copy constructor that copies the contents of the
// parameterized neighborhood into this neighborhood.
Neighborhood::Neighborhood(const Neighborhood &nh): vector<Neighbor>()
{
    reserve(MAX_NEIGHBORHOOD_SIZE);
    *this = nh;
}


// Copy constructor that copies the contents of the
// parameterized list of relationships into this neighborhood.
Neighborhood::Neighborhood(const vector<Relationship> &r)
{
    for (unsigned i = 0; i < r.size(); ++i)
    	addNbr(r[i], State());
}


// Copy constructor that copies the contents of the
// parameterized list of states into this neighborhood.
Neighborhood::Neighborhood(const vector<State> &s)
{
    for (unsigned i = 0; i < s.size(); ++i)
    	addNbr(Relationship(), s[i]);
}


// Destructor that clears this neighborhood.
Neighborhood::~Neighborhood()
{
}


// Attempts to add the parameterized neighbor to the neighborhood,
// returning true if successful, false otherwise.
bool Neighborhood::addNbr(const Neighbor n)
{
    if (size() < MAX_NEIGHBORHOOD_SIZE)
    {
        push_back(n);
        return true;
    }
    return false;
}


// Attempts to add a neighbor to the neighborhood
// based on the parameterized relationship and state,
// returning true if successful, false otherwise.
bool Neighborhood::addNbr(const Relationship r, const State s)
{
    return addNbr(Neighbor(r, s));
}


// Attempts to add a neighbor to the neighborhood
// based on the parameterized ID, state, and relationships,
// returning true if successful, false otherwise.
bool Neighborhood::addNbr(const int  id,      const State  s,
                          const Vector desired, const Vector actual)
{
    return addNbr(Neighbor(id, s, desired, actual));
}


// Attempts to remove the parameterized neighbor,
// returning true if successful, false otherwise.
bool Neighborhood::removeNbr(const Neighbor n)
{
    return removeNbr(n.ID);
}


// Attempts to remove the neighbor with the parameterized ID,
// returning true if successful, false otherwise.
bool Neighborhood::removeNbr(const int id)
{
    Neighbor nbr;
    for (unsigned i = 0; i < size(); ++i)
    {
        nbr = at(i);
        if (nbr.ID == id)
        {
            erase(begin() + i);
            return true;
        }
    }
    return false;
}


// Returns the list of neighbor relationships.
vector<Relationship> Neighborhood::getRelationships()
{
    vector<Relationship> rels;
    Neighbor currNbr;
    for (unsigned i = 0; i < size(); ++i)
    {
        currNbr = at(i);
        rels.push_back(currNbr);
    }
    return rels;
}


// Returns the list of neighbor states.
vector<State> Neighborhood::getStates()
{
    vector<State> states;
    Neighbor currNbr;
    for (unsigned i = 0; i < size(); ++i)
    {
        currNbr = at(i);
        states.push_back(currNbr);
    }
    return states;
}


// Returns the neighbor at the parameterized position in this neighborhood.
Neighbor* Neighborhood::getNbr(const int pos)
{
    if ((pos < 0) || (pos >= size()))
    	return NULL;
    return &at(pos);
}


// Returns the list of neighbors in this neighborhood.
vector<Neighbor> Neighborhood::getNbrs() const
{
    return *this;
}


// Returns the number of neighbors in this neighborhood.
int Neighborhood::getNNbrs() const
{
    return size();
}


// Updates the state of the parameterized neighbor,
// returning true if successful, false otherwise.
bool Neighborhood::updateNbr(Neighbor &n, const State &s)
{
    n = s;
    return true;
}


// Updates the state of the neighbor with the parameterized ID,
// returning true if successful, false otherwise.
bool Neighborhood::updateNbr(const int id, const State &s)
{
    Neighbor *nbr = nbrWithID(id);
    if (nbr == NULL)
    	return false;
    return updateNbr(*nbr, s);
}


// Returns true if the parameterized neighbor
// is in this neighborhood, false otherwise.
bool Neighborhood::isNbr(const Neighbor n)
{
    return isNbr(n.ID);
}


// Returns true if the neighbor with the parameterized ID
// is in this neighborhood, false otherwise.
bool Neighborhood::isNbr(const int id)
{
    Neighbor nbr;
    for (unsigned i = 0; i < size(); ++i)
    {
        nbr = at(i);
        if (nbr.ID == id) return true;
    }
    return false;
}


// Clears the neighbors in this neighborhood.
void Neighborhood::clearNbrs()
{
    clear();
}


// Returns a random neighbor in this neighborhood.
Neighbor* Neighborhood::anyNbr()
{
    srand(time(NULL));
    if (!empty())
    	return &at(irand(0, size()));
    return NULL;
}


// Returns the first neighbor in this neighborhood.
Neighbor* Neighborhood::firstNbr()
{
    if (empty())
    	return NULL;
    return &at(0);
}


// Returns the second neighbor in this neighborhood.//
Neighbor* Neighborhood::secondNbr()
{
    if (size() < 2)
    	return NULL;
    return &at(1);
}


// Returns the last neighbor in this neighborhood.
Neighbor* Neighborhood::lastNbr()
{
    if (empty())
    	return NULL;
    return &at(size() - 1);
}


// Returns the closest neighbor in this neighborhood
// as determined by the parameterized difference vector.
Neighbor* Neighborhood::closestNbr(const Vector v)
{
    Neighbor currNbr;
    float  minDist  = 0.0f, currDist = 0.0f;
    int    minIndex = ID_NO_NBR;

    for (unsigned i = 0; i < size(); ++i)
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
}


// Returns the furthest neighbor in this neighborhood
// as determined by the parameterized difference vector.
Neighbor* Neighborhood::furthestNbr(const Vector v)
{
    Neighbor currNbr;
    float  maxDist  = 0.0f, currDist = 0.0f;
    int    maxIndex = ID_NO_NBR;
    for (unsigned i = 0; i < size(); ++i)
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
}


// Returns the neighbor (in this neighborhood) with the parameterized ID.
Neighbor* Neighborhood::nbrWithID(const int id)
{
    Neighbor currNbr;
    for (unsigned i = 0; i < size(); ++i)
    {
        currNbr = at(i);
        if (currNbr.ID == id) return &at(i);
    }
    return NULL;
}


// Returns the neighbor (in this neighborhood) with the parameterized gradient.
Neighbor* Neighborhood::nbrWithGradient(const Vector grad)
{
    Neighbor currNbr;
    for (unsigned i = 0; i < size(); ++i)
    {
        currNbr = at(i);
        if (currNbr.gradient == grad) return &at(i);
    }
    return NULL;
}


// Returns the neighbor (in this neighborhood) with the minimum gradient
// distance as determined by the parameterized difference vector.
Neighbor* Neighborhood::nbrWithMinGradient(const Vector v)
{
    Neighbor currNbr;
    float  minGrad  = 0.0f, currGrad = 0.0f;
    int    minIndex = ID_NO_NBR;
    for (unsigned i = 0; i < size(); ++i)
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
}


// Returns the neighbor (in this neighborhood) with the maximum gradient
// distance as determined by the parameterized difference vector.
Neighbor* Neighborhood::nbrWithMaxGradient(const Vector v)
{
    Neighbor currNbr;
    float  maxGrad  = 0.0f, currGrad = 0.0f;
    int    maxIndex = ID_NO_NBR;
    for (unsigned i = 0; i < size(); ++i)
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
}


// Returns the neighbor (in this neighborhood) with the minimum step.
Neighbor* Neighborhood::nbrWithMinStep()
{
    Neighbor currNbr;
    int    minStep  = 0, currStep = 0;
    int    minIndex = ID_NO_NBR;
    for (unsigned i = 0; i < size(); ++i)
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
}


// Returns the neighbor (in this neighborhood) with the maximum step.
Neighbor* Neighborhood::nbrWithMaxStep()
{
    Neighbor currNbr;
    int    maxStep  = 0, currStep = 0;
    int    maxIndex = ID_NO_NBR;
    for (unsigned i = 0; i < size(); ++i)
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
}


// Sorts the neighborhood based upon neighbor ID.
void Neighborhood::sortByID()
{
    for (unsigned i = 0; i < size() - 1; ++i)
        for (unsigned j = i; j < size(); ++j)
            if (getNbr(i)->ID > getNbr(j)->ID)
            	swapNbrs(i, j);
}


// Sorts the neighborhood based upon neighbor gradient
// as determined by the parameterized difference vector.
void Neighborhood::sortByGradient(const Vector v)
{
    for (unsigned i = 0; i < size() - 1; ++i)
        for (unsigned j = i; j < size(); ++j)
            if ((getNbr(i)->gradient - v).magnitude() >
                (getNbr(j)->gradient - v).magnitude())
            	swapNbrs(i, j);
}


// Sorts the neighborhood based upon neighbor distance
// as determined by the parameterized difference vector.
void Neighborhood::sortByDistance(const Vector v)
{
    for (unsigned i = 0; i < size() - 1; ++i)
        for (unsigned j = i; j < size(); ++j)
            if ((getNbr(i)->relActual - v).magnitude() >
                (getNbr(j)->relActual - v).magnitude())
            	swapNbrs(i, j);
}


// Sorts the neighborhood based upon the angle of the
// neighbor-relative and the parameterized difference vector.
void Neighborhood::sortByAngle(const Vector v)
{
    for (unsigned i = 0; i < size() - 1; ++i)
        for (unsigned j = i; j < size(); ++j)
            if ((getNbr(i)->relActual - v).angle() >
                (getNbr(j)->relActual - v).angle())
            	swapNbrs(i, j);
}


// Sorts the neighborhood based upon the absolute value of the angle
// of the neighbor-relative and the parameterized difference vector.
void Neighborhood::sortByAbsAngle(const Vector v)
{
    for (unsigned i = 0; i < size() - 1; ++i)
        for (unsigned j = i; j < size(); ++j)
            if (abs((getNbr(i)->relActual - v).angle()) >
                abs((getNbr(j)->relActual - v).angle()))
            	swapNbrs(i, j);
}


// Swaps the neighbors at the parameterized indeces.
bool Neighborhood::swapNbrs(const int i, const int j)
{
    if ((i < 0) || (i >= size()) || (j < 0) || (j >= size()))
    	return false;
    Neighbor tempNbr = at(i);
    at(i)            = at(j);
    at(j)            = tempNbr;
    return true;
}

