#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <vector>
#include <numeric>

class Bin;

class Histogram
{
public:
    std::vector<double> data;
    int dimSize;
    int range;
    int rangePerBin;
    float rangePerBinInv;
    bool initialized;
    unsigned int lookUpTable1[256];
    unsigned int lookUpTable2[256];
    unsigned int lookUpTable3[256];

    Histogram(int _dimSize = 16, int _range = 256);
    ~Histogram(){}

    void insertValues(std::vector<unsigned char> & data1,
                      std::vector<unsigned char> & data2,
                      std::vector<unsigned char> & data3,
                      std::vector<double> & weight);

    double computeSimilarity(Histogram * hist);

    double getValue(int val1, int val2, int val3);
    void addValue(int val1, int val2, int val3, double weight);

    double getMin();
    void transformToWeights();
    void transformByWeight(double min);
    void multiplyByWeights(Histogram * hist);
    void adapt(Histogram hist, float height);

    void clear();
    void normalize();
    void addExpHist(double alpha, Histogram & hist);

};

#endif //HISTOGRAM_H
