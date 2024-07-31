#ifndef OBSERVER_H_
#define OBSERVER_H_

#include <vector>
#include <memory>
#include <algorithm>

////////////////////////////////
class IDataAnalyzer {
public:
    virtual void analyze(const std::vector<double>& data) = 0;
    virtual ~IDataAnalyzer() {}
};

////////////////////////////////
class DataProvider {
public:
    void setDataAndExecute(const std::vector<double>& data) {
        accelData = data;
        notify();
    }

    const std::vector<double>& getData() const {
        return accelData;
    }

    void registerAnalyzer(std::shared_ptr<IDataAnalyzer> analyzer) {
        analyzers.push_back(analyzer);
    }

private:
    void notify() {
        for (const auto& analyzer : analyzers) {
            analyzer->analyze(accelData);
        }
    }

    std::vector<double> accelData;
    std::vector<std::shared_ptr<IDataAnalyzer>> analyzers;
};

#endif // OBSERVER_H_