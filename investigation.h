#pragma once

#include <QObject>

#include "vector2d.h"

#include <vector>
#include <thread>
#include <mutex>


struct PolygonPoint;

constexpr int Steps = 200;

struct Rotation
{
    double cs;
    double sn;
};

class Investigation : public QObject
{
    Q_OBJECT


public:
    struct Result
    {
        std::vector<Vector2D> points;
        double square;
        std::vector<bool> image;
    };

    using Results = std::vector<Result>;

    Investigation(QObject* parent);

    ~Investigation();

    void geneticBranch(size_t index, double c = 1.);

    void genetic();

    double getSquare() const;

    static double perform2(Vector2D* offsets, Rotation* rotations, std::vector<bool>* image = nullptr);

    void perform2();

    void perform();

    PolygonPoint* getOptimalPolygon();

    const std::vector<bool>& getImage();

    double getSquare(int index);

    std::vector<QPointF> getPoints(int index, double w, double h);

    void stop();

    size_t getThreadsCount() const;

private:

    void notifyAboutChanging(int index = -1);

signals:
    void modified(int index);

private:

    std::vector<bool> image;

    std::mutex mutex;
    std::mutex fileMutex;

    std::vector<std::unique_ptr<std::thread>> threads;
    std::vector<Result> tempResults;

    Results results;

    Vector2D offsets[Steps + 1];
    Rotation rotations[Steps + 1];
    double square = -1;
    double scale = 1.f;
    PolygonPoint* polygon = nullptr;

    bool isDeleting = false;
    bool isTerminated = true;
};

