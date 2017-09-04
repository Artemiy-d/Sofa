#include "investigation.h"

#include <QDebug>

#include <QXmlStreamWriter>
#include <QXmlStreamReader>
#include <QFile>
#include <QPainter>
#include <QTextStream>

#include <time.h>
#include <set>
#include <math.h>
#include <cmath>
#include <bitset>

#include <algorithm>

#include <assert.h>

#include <map>
#include <vector>
#include <fstream>

#include <xmmintrin.h>
#include <emmintrin.h>
#include <smmintrin.h>
#include <pmmintrin.h>


template <typename T>
class Allocator
{

    struct Block
    {
        char _values[100 * sizeof(T)];
        T* _ptrs[100];
        size_t _count = 100;

        bool hasAllocated() const
        {
            return _count < 100;
        }

        Block()
        {
            for (int i = 0; i < 100; ++i)
            {
                _ptrs[i] = ((T*)_values) + i;
            }
        }

        ~Block()
        {

        }

        T* alloc()
        {
            if (_count == 0)
            {
                return nullptr;
            }

            return _ptrs[--_count];
        }

        bool free(T* ptr)
        {
            if (ptr >= ((T*)_values) && ptr < ((T*)_values) + 100)
            {
                _ptrs[_count++] = ptr;
                return true;
            }

            return false;
        }
    };

public:
    using value_type = T;

    T* allocate(size_t)
    {
        T* result = nullptr;
        auto it = std::find_if(_blocks.rbegin(), _blocks.rend(), [&result](const
std::unique_ptr<Block>& block)
        {
            result = block->alloc();
            return !!result;
        });

        if (!result)
        {
            _blocks.emplace_back(new Block);
            result = _blocks.back()->alloc();
        }
        else if (it != _blocks.rbegin())
        {
            std::swap(*it, *_blocks.rbegin());
        }

        return result;
    }

    void deallocate(T* ptr, size_t)
    {
        auto found = std::find_if(_blocks.rbegin(), _blocks.rend(), [ptr](const
std::unique_ptr<Block>& block)
        {
            return block->free(ptr);
        });

        if (!(*found)->hasAllocated() && found != _blocks.rbegin() &&
!_blocks.back()->hasAllocated())
        {
            _blocks.erase(std::next(found).base());
        }
    }

    template <typename... Args>
    void construct(T* v, Args&&... args)
    {
        new (v) T(std::forward<Args>(args)...);
    }

    template <typename... Args>
    T* construct1(Args&&... args)
    {
        T* result = nullptr;
        auto it = _blocks.rbegin();
        for (; !result && it != _blocks.rend(); ++it)
        {
            result = (*it)->alloc();
        }

        if (!result)
        {
            _blocks.emplace_back(new Block);
            result = _blocks.back()->alloc();
        }
        else if (it != _blocks.rbegin())
        {
            std::swap(*it, *_blocks.rbegin());
        }

        return new (result) T(std::forward<Args>(args)...);
    }

    void destroy(T* ptr)
    {
        ptr->~T();
    }

    void destroy1(T* ptr)
    {
        auto found = std::find_if(_blocks.rbegin(), _blocks.rend(), [ptr](const
std::unique_ptr<Block>& block)
        {
            return block->free(ptr);
        });

        ptr->~T();

        if (!(*found)->hasAllocated() && found != _blocks.rbegin() &&
!_blocks.back()->hasAllocated())
        {
            _blocks.erase(std::next(found).base());
        }
    }

    size_t max_size() const
    {
        return 1;
    }
private:
    std::vector<std::unique_ptr<Block>> _blocks;
};

struct Transformation
{
    Vector2D _offset;
    double _cosAngle;
    double _sinAngle;
};

Vector2D transform(Vector2D point, const Transformation& t)
{
    point -= t._offset;
    return
    {
        point.x() * t._sinAngle + point.y() * t._cosAngle,
        point.y() * t._cosAngle + point.x() * t._sinAngle
    };
}

bool testPoint(Vector2D point, const Transformation& t)
{
    point = transform(point, t);
    return point.x() < 0 && point.y() < 1 && point.y() > 0;
}

struct Moving
{
    Transformation _transformations[100];

    double getSquare() const
    {
        return 0.f;
    }
};


#define EPS 0.0000001

struct PolygonPoint;



struct Intersection
{
    PolygonPoint* polygonPoint;
    Vector2D point;
    double distance;

    int rayNumber;

    Intersection* next;
};



struct PolygonPoint
{
    Vector2D point;
    PolygonPoint* next = 0;
    PolygonPoint* ref = 0;
};

struct Ray
{
    Vector2D dir;
    Vector2D point;
};

struct Space
{
    Ray rays[4];
};

constexpr int maxRayLen = 6;

Vector2D getTransformed(const Vector2D& point, const Rotation& r, const Vector2D&
offset)
{
    return {point.x() * r.cs - point.y() * r.sn + offset.x(),
                point.x() * r.sn + point.y() * r.cs + offset.y()};
}

Vector2D getRTransformed(const Vector2D& point, const Rotation& r, const Vector2D& offset)
{
    auto p = point - offset;
    return {p.x() * r.cs + p.y() * r.sn,
                -p.x() * r.sn + p.y() * r.cs};
}

void initSpace(Space& space, const Rotation& r, const Vector2D& offset)
{
    space.rays[0].point = getTransformed({1, 1 - maxRayLen}, r, offset);
    space.rays[0].dir = {-r.sn, r.cs};

    space.rays[1].point = getTransformed({1, 1}, r, offset);
    space.rays[1].dir = {-r.cs, -r.sn}; // -1, 0

    space.rays[2].point = getTransformed({-maxRayLen, 0}, r, offset);
    space.rays[2].dir = {r.cs, r.sn};  // 1, 0

    space.rays[3].point = offset;
    space.rays[3].dir = {r.sn, -r.cs};
}

bool intersection(const Ray& ray, const Vector2D& a, const Vector2D& b, Vector2D& result, double& dist)
{
    if (std::abs(a.x() - b.x()) < EPS && std::abs(a.y() - b.y()) < EPS)
    {
        return false;
    }

    const auto dir = (b - a).normalized();
    const Vector2D orth(-dir.y(), dir.x());
    const auto d = Vector2D::dotProduct(orth, a - ray.point);
    const auto e = Vector2D::dotProduct(orth, ray.dir);

    if (fabs(e) < EPS)
    {
        return false;
    }
    else
    {
        dist = d / e;
    }

    if (dist < -EPS || dist > maxRayLen + EPS)
    {
        return false;
    }

    result = ray.point + ray.dir * dist;

    if (dist < 0)
    {
        dist = 0;
    }
    else if (dist > maxRayLen)
    {
        dist = maxRayLen;
    }

    if (std::abs(result.x() - b.x()) < EPS && std::abs(result.y() - b.y()) < EPS)
    {
        result = b;
        return true;
    }

    if (std::abs(result.x() - a.x()) < EPS && std::abs(result.y() - a.y()) < EPS)
    {
        result = a;
        return true;
    }

    result = a + Vector2D::dotProduct(result - a, dir) * dir;

    return Vector2D::dotProduct(result - a, b - result) >= 0.;
}

void check(PolygonPoint* pol)
{
    auto p = pol;
    qDebug();
    do
    {
       // qDebug() << "check: " << p->point;
        p = p->next;
    } while (p != pol);
}

PolygonPoint* mark(PolygonPoint* polygon, const Ray* rays)
{
    Vector2D inter;

    Intersection* firstIntersection = nullptr;
    Intersection* lastIntersection = nullptr;

   // std::vector<std::pair<double, >>
    int ic = 0;

    int ics[4] = {0, };

    for (int i = 0; i < 4; ++i)
    {
        PolygonPoint* pol = polygon;
        do
        {
            double dist;

            if (i == 2)
            {
                int x = 0;
                ++x;
            }

            if (intersection(rays[i], pol->point, pol->next->point, inter, dist))
            {
                ++ic;
                ++ics[i];
                auto in = new Intersection;
                pol->ref = pol;
                in->polygonPoint = pol;
                in->point = inter;
                if (dist >maxRayLen - 0.1)
                {
                    int x = 0;
                    ++x;
                }
                in->distance = dist + i * maxRayLen;
                in->next = nullptr;
                in->rayNumber = i;

                qDebug() << "in " << i << " " << inter.x() << inter.y();

                if (!firstIntersection)
                {
                    firstIntersection = lastIntersection = in;
                }
                else
                {
                    lastIntersection->next = in;
                    lastIntersection = in;
                }
            }

            pol = pol->next;
        } while (pol != polygon);

        if (lastIntersection)
        {
            for (auto a = firstIntersection; a->next; a = a->next)
            {
                for (auto b = a; b->next; )
                {
                    auto n = b->next;
                    if (b->distance > n->distance)
                    {
                        std::swap(n->distance, b->distance);
                        std::swap(n->point, b->point);
                        std::swap(n->polygonPoint, b->polygonPoint);
                    }

                    b = n;
                }
            }
        }
    }

    qDebug() << "ics " << ics[0] << ics[1] << ics[2] << ics[3];

    Intersection* prevInter = nullptr;
    PolygonPoint* lastPolygon = nullptr;
    Vector2D lastInterPoint;


    for (auto a = firstIntersection; a; a = a->next)
    {
        auto p = a->polygonPoint;

        check(p);
    }

    for (auto a = firstIntersection; a; a = a->next)
    {
        if (prevInter && std::abs(a->distance - prevInter->distance) < EPS)
        {
            if (prevInter->rayNumber == a->rayNumber)
            continue;
        }

        auto p = a->polygonPoint->ref;

        check(p);


        if (lastPolygon)
        {

            if (lastPolygon == p)
            {
                qDebug() << "lastPolygon == p";
                {
                    auto newP = new PolygonPoint;
                    newP->next = p->next;
                    p->ref = newP;
                    p = newP;
                }

                {
                    auto newP = new PolygonPoint;
                    newP->next = p;
                    //newP->point = lastInterPoint;
                    lastPolygon->next = newP;
                }
            }
            else if (lastPolygon->next == p)
            {
                qDebug() << "lastPolygon->next == p";

                auto newP = new PolygonPoint;
                newP->next = p->next;
                p->ref = newP;
                p = newP;

            }
            else
            {
                auto i = lastPolygon->next->next;
                lastPolygon->next->next = p;
               // qDebug() << "cb " << lastPolygon->point.x() << lastPolygon->point << lastPolygon->next->point;
                while (i != p)
                {
                    qDebug() << "1";
                    auto next = i->next;
                 //   assert(i != lastPolygon);
                    for (auto b = firstIntersection; b; b = b->next)
                    {
                        assert(b->polygonPoint != i);
                        assert(b->polygonPoint->ref != i);
                    }
                    delete i;
                    i = next;

                    qDebug() << "2";
                }

                qDebug() << "ce";

            }

            lastPolygon->next->next = p;

            //qDebug() << "lastPolygon->point " << lastPolygon->point;
          //  qDebug() << "lastPolygon->next->point " << lastPolygon->next->point;
            lastPolygon->next->point = lastInterPoint;

          //  qDebug() << "p->point " << p->point;
            p->point = a->point;

           // qDebug() << "a->point " << a->point;

            check(p);

            if (a->rayNumber != prevInter->rayNumber)
            {
                auto newP = new PolygonPoint;
                newP->next = p;
                qDebug() << "rn " << a->rayNumber;
                newP->point = rays[a->rayNumber].point;

                lastPolygon->next->next = newP;
            }

            check(p);

            lastPolygon = nullptr;

             check(p);

             qDebug() << "bu";
        }
        else
        {
            lastPolygon = p;

           // assert(last == polygon);
            lastInterPoint = a->point;

            qDebug() << "bu1";

            check(p);
        }

        check(p);

        polygon = p;

        prevInter = a;
    }

    assert(lastPolygon == nullptr);
    check(polygon);

    qDebug() << "e";

    return polygon;
}

void drawPolygon(QPainter& painter, double w, double h, PolygonPoint* polygon)
{
    painter.setPen(Qt::blue);

    QPointF room[] =
    {
        {0, h / 2},
        {w / 2, h / 2},
        {w / 2, h / 2},
        {w / 2, h},

        {0, h / 2 - w / 8},
        {w / 2 + w / 8, h / 2 - w / 8},
        {w / 2 + w / 8, h / 2 - w / 8},
        {w / 2 + w / 8, h},
    };

    painter.drawLines(room, 8);

    painter.setPen(Qt::black);

    check(polygon);
    std::vector<QPointF> points;
    auto p = polygon;
    do
    {
        points.emplace_back(w / 2 + p->point.x() * w / 8, h / 2 - p->point.y() * w / 8);

        p = p->next;
    } while (p != polygon);

    painter.drawPolygon(points.data(), points.size());
}

PolygonPoint* createBasePolygon()
{
    static const Vector2D points[] =
    {
        {1, 0},
        {1, 1},
        {-3, 1},
        {-3, 0}
    };

    PolygonPoint* polygonPoints[4];

    for (int i = 0; i < 4; ++i)
    {
        polygonPoints[i] = new PolygonPoint;
        polygonPoints[i]->point = points[i];
    }

    for (int i = 0; i < 4; ++i)
    {
        polygonPoints[i]->next = polygonPoints[(i + 1) % 4];
    }

    return polygonPoints[0];
}


double getPolygonSquare(PolygonPoint* )
{
    return 1;
}

constexpr double maxSofaLen = 4;

int ImageH = 200;
int ImageW = static_cast<int>(ImageH * maxSofaLen);

template <typename T>
T rand(Generator& generator, const T& a, const T& b)
{
    return typename std::conditional<
            std::is_floating_point<T>::value,
            std::uniform_real_distribution<T>,
            std::uniform_int_distribution<T>
            >::type(a, b)(generator);
}

template <typename T>
T rand(Generator& generator, const T& a)
{
    return rand(generator, a, -a);
}

inline bool chance(Generator& generator, int f)
{
    return generator() % f == 0;
}


int getIndex(int i)
{
    return i;
}

int getRIndex(int i)
{
    return Steps - i - 1;
}

void modifyPopulation4(Generator& generator, const Vector2D* in, Vector2D* out, double c)
{
    Vector2D offset;
    Vector2D delta;

    int i = 1;


    for (; i <= Steps; ++i)
    {
        if (chance(generator, 60))
        {
            delta.setX(delta.x() + rand(generator, c * 0.02 * 0.2));
        }

        if (chance(generator, 60))
        {
            delta.setY(delta.y() + rand(generator, c * 0.01 * 0.2));
        }

        offset += 0.15 * delta;
        delta *= 0.85;

        out[i] = in[i] + offset;
        if (i > 0 && out[i].x() > out[i - 1].x())
        {
            out[i].setX(out[i - 1].x());
        }

        if (out[i].y() < 0)
        {
            out[i].setY(0);
        }

        offset *= 0.9;
    }
}

void modifyPopulation3(Generator& generator, const Vector2D* in, Vector2D* out, double c)
{
  //  out[Steps - 1].setX(in[Steps - 1].x() + 0.2 * (rand() - RAND_MAX / 2) / RAND_MAX);
  //  double c = out[Steps - 1].x() / in[Steps - 1].x();

    Vector2D offset;
    Vector2D delta;

    int i = 1;


    for (; i <= Steps; ++i)
    {
        if (chance(generator, 100))
        {
            delta.setX(delta.x() + rand(generator, c * 0.03 * 0.25));
        }

        if (chance(generator, 90))
        {
            delta.setY(delta.y() + rand(generator, c * 0.015 * 0.25));
        }

        offset += 0.1 * delta;
        delta *= 0.9;

        out[i] = in[i] + offset;
        if (i > 0 && out[i].x() > out[i - 1].x())
        {
            out[i].setX(out[i - 1].x());
        }

        if (out[i].y() < 0)
        {
            out[i].setY(0);
        }

        offset *= 0.95;
    }
}

void modifyPopulation2(Generator& generator, const Vector2D* in, Vector2D* out, double c)
{
    Vector2D offset;
    Vector2D delta;

    int i = 1;


    for (; i <= Steps; ++i)
    {
        if (chance(generator, 90))
        {
            delta.setX(delta.x() + rand(generator, c * 0.03 * 0.25));
        }

        if (chance(generator, 90))
        {
            delta.setY(delta.y() + rand(generator, c * 0.015 * 0.25));
        }

        offset += 0.05 * delta;
        delta *= 0.95;

        out[i] = in[i] + offset;
        if (i > 0 && out[i].x() > out[i - 1].x())
        {
            out[i].setX(out[i - 1].x());
        }

        if (out[i].y() < 0)
        {
            out[i].setY(0);
        }

        offset *= 0.98;
    }
}

void rotate(Vector2D* out)
{
    auto first = out[0].x();
    auto last = out[Steps].x();
    for (int i = 0; i < (Steps + 1) / 2; ++i)
    {
        auto temp = out[i];
        out[i].setY(out[Steps - i].y());
        out[Steps - i].setY(temp.y());

        out[i].setX(last - out[Steps - i].x() + first);
        out[Steps - i].setX(last - temp.x() + first);
    }
}


void selfCross(Vector2D* out)
{
    const auto deltaX = out[Steps].x() - out[0].x();
    for (int i = 0; i < (Steps + 1) / 2; ++i)
    {
        out[i].setY((out[Steps - i].y() + out[i].y()) * 0.5);
        out[Steps - i].setY(out[i].y());

        out[i].setX((deltaX - out[Steps - i].x() + out[i].x()) * 0.5);
        out[Steps - i].setX(deltaX - out[i].x());
    }
}


void filterLow(Vector2D* out, double a)
{
    assert(a >= 0 && a < 1);
    a = 1. - a;
    const auto b = (1. - a) * 0.5;
    auto prevY = out[0].y();
    for (int i = 1; i < Steps; ++i)
    {
        auto valueY = a * out[i].y() + (prevY + out[i + 1].y()) * b;
        prevY = out[i].y();
        out[i].setY(valueY);
    }
}

void stretchOffsets(Vector2D* offsets, double c)
{
    c += 1.;
    const double fulcrum = offsets[0].x();
    for (int i = 0; i <= Steps; ++i)
    {
        offsets[i].setX((offsets[i].x() - fulcrum) * c);
    }
}


void modifyPopulation(Generator& generator, const Vector2D* in, Vector2D* out, double c)
{
    for (int i = 0; i <= Steps; ++i)
    {
        out[i] = in[i];
    }

    if (chance(generator, 2))
    {
        stretchOffsets(out, rand(generator, c * 0.02));
    }

    if (chance(generator, 2))
    {
        filterLow(out, rand(generator, 0., 0.4));
    }
}

void mix(const Vector2D* first, const Vector2D* second, Vector2D* out, double c)
{
    const auto c1 = 0.5 + c;
    const auto c2 = 1. - c1;

    for (int i = 0; i <= Steps; ++i)
    {
        out[i] = c1 * first[i] + c2 * second[i];
    }
}

void generateOffsets(Vector2D* offsets)
{
    const double len = 1.5;
    const double first = 0.;//-0.08f * rand() / RAND_MAX;
    for (int i = 0; i <= Steps; ++i)
    {
        offsets[i].setX(first - i * len / Steps);
        offsets[i].setY(0);
       // if (i == 9)
       // offsets[i].setY(0.2);
    }
}


void checkOffsets(Vector2D* offsets)
{
        auto delta = offsets[0] - Vector2D(0, 0);
        for (int i = 0; i <= Steps; ++i)
        {
            offsets[i] -= delta;
        }
}

static const QString TagMain("main");
static const QString TagPoints("points");
static const QString TagResult("result");
static const QString TagScale("scale");

void saveOffsets(Vector2D* offsets, double scale)
{
    QFile file("result.xml");
    if (!file.open(QFile::WriteOnly | QFile::Text | QFile::Truncate))
    {
        return;
    }

    QXmlStreamWriter writer(&file);
    writer.setAutoFormatting(true);

    auto writeResult = [&writer, offsets]()
    {
        writer.writeStartElement(TagResult);
        writer.writeStartElement(TagPoints);

        QString s;
        QTextStream ts(&s);
        ts.setRealNumberPrecision(15);

        for (int i = 0; i <= Steps; ++i)
        {
            ts << offsets[i].x() << ' ' << offsets[i].y() << ' ';
        }

        writer.writeCharacters(s);

        writer.writeEndElement();
        writer.writeEndElement();
    };

    writer.writeStartDocument();
    writer.writeStartElement(TagMain);

    writer.writeStartElement(TagScale);
    writer.writeCharacters(QString::number(scale));
    writer.writeEndElement();

    writeResult();

    writer.writeEndElement();
    writer.writeEndDocument();

    file.close();
}


bool loadOffsets(std::vector<Vector2D>& offsets, double& scale)
{
    QFile file("result.xml");
    if (file.open(QFile::ReadOnly | QFile::Text))
    {
        static const std::set<QStringRef> tags =
        {
            &TagMain,
            &TagResult,
            &TagPoints,
            &TagScale
        };

        QXmlStreamReader reader(&file);

        std::vector<const QString*> tagsStack;
        QString text;

        auto checkStack = [&](std::initializer_list<const QString*> tagsList)
        {
            auto result = tagsStack.size() == tagsList.size() &&
                          std::equal(tagsList.begin(), tagsList.end(), tagsStack.begin());

            if (result)
            {
                tagsStack.pop_back();
                text = reader.readElementText();
            }

            return result;
        };

        while (!reader.atEnd())
        {
            switch (reader.readNext())
            {
                case QXmlStreamReader::StartElement:
                {
                    auto found = tags.find(reader.name());
                    tagsStack.push_back(found != tags.end() ? found->string() : nullptr);

                    if (checkStack({&TagMain, &TagResult, &TagPoints}))
                    {
                        QTextStream stream(&text);
                        stream.setRealNumberPrecision(15);

                        double x = 0, y = 0;
                        while (!stream.atEnd())
                        {
                            stream >> x >> y;
                            stream.skipWhiteSpace();

                            offsets.emplace_back(x, y);
                        }
                    }
                    else if (checkStack({&TagMain, &TagScale}))
                    {
                        QTextStream stream(&text);
                        stream.setRealNumberPrecision(15);
                        stream >> scale;
                    }
                    break;
                }
                case QXmlStreamReader::EndElement:
                {
                    if (tagsStack.empty())
                    {
                        return false;
                    }
                    tagsStack.pop_back();
                    break;
                }
                case QXmlStreamReader::Invalid:
                    return false;
                default:
                    break;
            }
        }

        assert(offsets.size() == Steps + 1);

      //   reader.readNextStartElement()

        return true;
    }

    file.setFileName("result.txt");
    if (file.open(QFile::ReadOnly | QFile::Text))
    {
        QTextStream stream(&file);

        double x = 0., y = 0.;

        while (!stream.atEnd())
        {
            stream >> x;

            if (stream.atEnd())
            {
                break;
            }

            stream >> y;

            offsets.emplace_back(x, y);
        }

        qDebug() << "loaded size: " << offsets.size();

        assert(offsets.size() == Steps + 1);

        return true;
    }

    return false;
}



Investigation::Investigation(QObject* parent) :
    QObject(parent),
    threadData(std::thread::hardware_concurrency())
{
    for (int i = 0; i <= Steps; ++i)
    {
        rotations[i].cs = cos(i * M_PI_2 / Steps);
        rotations[i].sn = sin(i * M_PI_2 / Steps);

        offsets[i].setX(-i * 1. / Steps);
       // if (i == 9)
       // offsets[i].setY(0.2);
    }

    image.resize(ImageW * ImageH);
}

Investigation::~Investigation()
{
    isDeleting = true;
    stop();
}

void Investigation::geneticBranch(size_t index, double c)
{
    Result& result = threadData[index].result;
    auto& generator = threadData[index].generator;

    constexpr int popCount = 20;

    Results p(popCount, { result.points, result.square, std::vector<bool>()});

    for (int i = 0; i < popCount; ++i)
    {
        p[i].square = perform2(p[i].points.data(), rotations);
    }

    auto oldMax = p[0].square;
    int counter = 0;
    int fullCounter = 0;

    decltype(&modifyPopulation) m[] = {
        modifyPopulation2,
        modifyPopulation,
        modifyPopulation3,
        modifyPopulation4};

    int mIndex = 0;

    for (int t = 0; fullCounter < 140 && !isTerminated; ++t)
    {
        if (t % 20 == 0)
        {
            qDebug() << "genetic iterations " << index << ":" << t << p.front().square;

          //  auto r = p.front();

            {
                std::lock_guard<std::mutex> guard(mutex);

                result = p.front();
                perform2(result.points.data(), rotations, &result.image);
            }



            notifyAboutChanging(index);
        }


        if (counter > 20)
        {

            mIndex = (mIndex + 1) % (sizeof(m) / sizeof(m[0]));
            counter = 0;
            c *= 0.95;

            qDebug() << "change mod " << index << ":" << mIndex << c;
        }



        for (int i = 2; i < popCount; ++i)
        {
         //   cross(p[0].first.data(), p[1].first.data(), p[i].first.data());

            (*m[mIndex])(generator, p[i % 2].points.data(), p[i].points.data(), c);

            p[i].points.back().setY(0);

            if (chance(generator, 5))
            {
                selfCross(p[i].points.data());
            }

            p[i].square = perform2(p[i].points.data(), rotations);
        }

        std::sort(p.begin(), p.end(), [](const Result& a, const Result& b)
        {
            return b.square < a.square;
        });

        rotate(p[generator() % 2].points.data());

        if (oldMax > p[0].square - EPS)
        {
            ++counter;
            ++fullCounter;
        }
        else
        {
            counter = 0;
        }

        oldMax = p[0].square;
    }

    result = std::move(p.front());
}

void Investigation::genetic()
{
    stop();

    Result res;
    scale = 1.;
    if (!loadOffsets(res.points, scale))
    {
        res.points.resize(Steps + 1);
        generateOffsets(res.points.data());
    }
    else
    {
        checkOffsets(res.points.data());
    }


    res.square = perform2(res.points.data(), rotations, &res.image);
    results.push_back(std::move(res));

    notifyAboutChanging();

    isTerminated = false;

    for (size_t currentIndex = 0; currentIndex < threadData.size(); ++currentIndex)
    {
        auto& thrData = threadData[currentIndex];
        thrData.result = results.front();
        thrData.generator.seed(time(nullptr) + currentIndex);

       thrData.thread = std::thread([this, currentIndex, &thrData]()
       {
           while (!isTerminated)
           {
               if (!chance(thrData.generator, 12))
               {
                   stretchOffsets(thrData.result.points.data(), rand(thrData.generator, 0.2 * sqrt(scale)));

                   for (int i = 0; i < 8; ++i)
                   {
                       filterLow(thrData.result.points.data(), rand(thrData.generator, 0., 0.8));
                   }
               }
               else if (chance(thrData.generator, 2))
               {
                   mix(thrData.result.points.data(), results.front().points.data(), thrData.result.points.data(), rand(thrData.generator, 0.2));
               }

               geneticBranch(currentIndex, rand(thrData.generator, 0.9, 1.1) * scale);

               bool isResultModified = false;
               double oldScale = 0.;

               {
                    std::lock_guard<std::mutex> guard(mutex);

                    if (thrData.result.square > results.front().square)
                    {
                        results.front() = thrData.result;
                        perform2(results.front().points.data(), rotations, &results.front().image);
                        isResultModified = true;
                    }
                    else
                    {
                        thrData.result = results.front();
                    }

                    oldScale = scale;
                    scale *= 0.996;
               }


               if (isResultModified)
               {
                   {
                       std::lock_guard<std::mutex> guard(fileMutex);
                       saveOffsets(thrData.result.points.data(), oldScale);
                   }

                   qDebug() << "new maximum: " << thrData.result.square;
                   notifyAboutChanging();
               }
           }
       });
    }

    saveOffsets(results.front().points.data(), scale);
}

double Investigation::perform2(Vector2D* offsets, Rotation* rotations, std::vector<bool>* image)
{
    int t = 0;
    int c = 0;

    if (image && image->size() != size_t(ImageW * ImageH))
    {
        image->resize(ImageW * ImageH);
    }

    for (int i = 0; i < ImageW; ++i)
        for (int j = 0; j < ImageH; ++j)
        {

            const Vector2D point((double(i) + 0.5) * maxSofaLen / double(ImageW) - maxSofaLen + 1,
                                  (double(j) + 0.5) / ImageH);


#ifdef QT_DEBUG
            auto check = [&](int index)
            {
                const auto p = getRTransformed(point, rotations[index], offsets[index]);
                return p.x() < 1 && p.y() < 1 && !(p.x() < 0. && p.y() < 0.);
            };
#else
            auto oneone = _mm_set_pd(1., 1.);
            auto psse = _mm_loadu_pd((double*)&point);
            auto check = [&](int index)
            {
                auto r = _mm_loadu_pd((double*)&rotations[index]);

                auto delta = _mm_sub_pd(
                            psse,
                            _mm_loadu_pd((double*)(offsets + index)));

                auto r1 = _mm_shuffle_pd(r, r, 1);

                auto p = _mm_addsub_pd(
                            _mm_mul_pd(_mm_shuffle_pd(delta, delta, 3), r),
                            _mm_mul_pd(_mm_shuffle_pd(delta, delta, 0), r1)
                            );

                auto res = _mm_andnot_pd(_mm_cmpnge_pd(p, _mm_setzero_pd()),
                                         _mm_cmpnge_pd(p, oneone));

                return _mm_movemask_pd(res) & 3;
            };
#endif

            bool f = Steps % 2 == 0 && check(Steps / 2);

            for (int k = 0; k < (Steps + 1) / 2 && f; ++k)
            {
                f = check(k) && check(Steps - k);
            }

            c += f;
            if (image)
            {
                (*image)[t++] = f;
            }
        }

    return maxSofaLen * double(c) / (ImageH * ImageW);
}

void Investigation::perform2()
{
    square = perform2(offsets, rotations, &image);
}

void Investigation::perform()
{
    auto p = createBasePolygon();

    Space space;

    for (int i = 0; i < Steps; ++i)
    {
        initSpace(space, rotations[i], offsets[i]);

        p = mark(p, space.rays);
    }

    auto s = getPolygonSquare(p);
    if (s > square)
    {
        square = s;
        polygon = p;
    }
}

PolygonPoint* Investigation::getOptimalPolygon()
{
    return polygon;
}

const std::vector<bool>& Investigation::getImage()
{
    return image;
}

double Investigation::getSquare(int index)
{
    std::lock_guard<std::mutex> guard(mutex);

    const auto& r = index < 0 ? results.front() : threadData[index].result;

    return r.square;
}

std::vector<QPointF> Investigation::getPoints(int index, double w, double h)
{
    std::lock_guard<std::mutex> guard(mutex);

    const auto& r = index < 0 ? results.front() : threadData[index].result;

    constexpr int sc = 4;

    std::vector<QPointF> result;

    int t = 0;
    for (int i = 0; i < ImageW; ++i)
        for (int j = 0; j < ImageH; ++j)
        {
            if (r.image[t++])
            {
                result.emplace_back(
                            w / 2 + ((i + 0.5) * maxSofaLen / double(ImageW) - maxSofaLen + 1.5) * w / sc,
                            h / 2 - ((j + 0.5) / double(ImageH)) * w / sc);
            }
        }

    return result;
}


void Investigation::stop()
{
    if (!isTerminated)
    {
        isTerminated = true;
        for (auto& thrData : threadData)
        {
            thrData.thread.join();
        }
    }
}

void Investigation::notifyAboutChanging(int index)
{
    if (!isDeleting)
    {
        emit modified(index);
    }
}

size_t Investigation::getThreadsCount() const
{
    return threadData.size();
}

