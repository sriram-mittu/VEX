#include "main.h"
using namespace std;

const double spacing = 3.0;
const double trackWidth = 16;
const double epsilon = 0.000000000000001;
const double maxAccel = 12, maxVelocity = 100;
const double b = 0.8, a = 1 - b, tolerance = 0.001;
const double kV = 1 / maxVelocity, kA = 0.002, kP = 0.01;

vector<vector<double>> injectPoints(vector<vector<double>> wp) {
    vector<vector<double>> newPoints = {};
    for (int i = 0; i < wp.size() - 1; i++) {
        double disX = wp[i + 1][0] - wp[i][0];
        double disY = wp[i + 1][1] - wp[i][1];

        double distance = sqrt((disX * disX) + (disY * disY));
        double fit = ceil(distance / spacing);
        double angle = atan2(disY, disX);

        double vecX = spacing * cos(angle);
        double vecY = spacing * sin(angle);

        for (int j = 0; j < fit; j++) newPoints.push_back({wp[i][0] + vecX * j, wp[i][1] + vecY * j});
    }
    newPoints.push_back(wp[wp.size() - 1]);
    return newPoints;
}

vector<vector<double>> smoothPath(vector<vector<double>> wp) {
    vector<vector<double>> tempArray = wp;
    bool flag = true;
    double change = tolerance;
    while (flag) {
        change = 0;
        for (int i = 1; i < wp.size() - 1; i++) {
            for (int j = 0; j < wp[i].size(); j++) {
                double aux = tempArray[i][j];
                tempArray[i][j] += a * (wp[i][j] - tempArray[i][j]) + b * (tempArray[i - 1][j] + tempArray[i + 1][j] - (2 * tempArray[i][j]));
                change += abs(aux - tempArray[i][j]);
                if (change < tolerance) {
                    flag = false;
                    break;
                }
            }
            if (!(flag)) break;
        }
    }
    return tempArray;
}

double curvatureAtIndex(int i, vector<vector<double>> sa) {
    double x1 = sa[i - 1][0] + epsilon, x2 = sa[i + 0][0], x3 = sa[i + 1][0];
    double y1 = sa[i - 1][1], y2 = sa[i + 0][1], y3 = sa[i + 1][1];

    double k1 = 0.5 * (x1 * x1 + y1 * y1 - x2 * x2 - y2 * y2) / (x1 - x2 + epsilon);
    double k2 = (y1 - y2) / (x1 - x2 + epsilon);

    double b = 0.5 * (x2 * x2 - 2 * x2 * k1 + y2 * y2 - x3 * x3 + 2 * x3 * k1 - y3 * y3) / (x3 * k2 - y3 + y2 - x2 * k2 + epsilon);
    return hypot((x1 - (k1 - k2 * b)), (y1 - b));
}

vector<vector<double>> addValues(vector<vector<double>> sa) {
    vector<vector<double>> smoothed = sa;
    smoothed[0].push_back(maxVelocity);

    for (int k = 1; k < smoothed.size() - 1; k++) {
        double deltaX = smoothed[k + 1][0] - smoothed[k][0];
        double deltaY = smoothed[k + 1][1] - smoothed[k][1];
        double dist = hypot(deltaY, deltaX);
        double curv = curvatureAtIndex(k, smoothed) + epsilon;
        smoothed[k].push_back(min(min(maxVelocity, curv), sqrt(pow(smoothed[k + 1][3], 2) + 2 * maxAccel * dist)));
    }

    smoothed[smoothed.size()-1].push_back(maxVelocity);
    return smoothed;
}

int closestPoint(vector<vector<double>> sa) {
    double minimumDistance = hypot(sa[0][0] - posX, sa[0][1] - posY);
    double location = 0;

    for (int i = 0; i < sa.size(); i++) {
        double currentDistance = hypot(sa[i][0] - posX, sa[i][1] - posY);
        if (currentDistance < minimumDistance) {
            minimumDistance = currentDistance;
            location = i;
        }
    }
    return location;
}

vector<double> lookaheadPoint(vector<vector<double>> sa, double lookaheadDistance) {
    for (int i = sa.size() - 1; i > 0; i--) {
        vector<double> E = sa[i - 1];
        vector<double> L = sa[i];
        vector<double> D = {L[0] - E[0], L[1] - E[1]};
        vector<double> F = {E[0] - posX, E[1] - posY};

        double a = D[0] * D[0] + D[1] * D[1];
        double b = 2 * (F[0] * D[0] + F[1] * D[1]);
        double c = (F[0] * F[0] + F[1] * F[1]) - (pow(lookaheadDistance, 2));
        double discriminant = (b * b) - (4 * a * c);

        if (discriminant >= 0) {
            discriminant = sqrt(discriminant);
            double t1 = (-b - discriminant) / (2 * a);
            double t2 = (-b + discriminant) / (2 * a);
            if (t1 >= 0 && t1 <= 1) return {sa[i][0] + t1 * D[0], sa[i][1] + t1 * D[1]};
            if (t2 >= 0 && t2 <= 1) return {sa[i][0] + t2 * D[0], sa[i][1] + t2 * D[1]};
        }
    }
    return sa[closestPoint(sa)];
}

double lookaheadCurvature(vector<double> lookaheadPoint, double lookaheadDistance) {
    double side = 1;
    double value = sin(pi / 2 - radians) * (lookaheadPoint[0] - posX) - cos(pi / 2 - radians) * (lookaheadPoint[1] - posY);
    if (value < 0) side = -1;
    if (value > 0) side = 1;

    double a = -tan(pi / 2 - radians);
    double x = abs(a * lookaheadPoint[0] + lookaheadPoint[1] + (tan(3.1415 / 2 - radians) * posX - posY)) / sqrt(a * a + 1);
    return side * (2 * x / (lookaheadDistance * lookaheadDistance));
}

void followPath(vector<vector<double>> wp, double lookaheadDistance) {
    vector<vector<double>> smoothed = addValues(smoothPath(injectPoints(wp)));
    setCoast();

    double lastLeft = 0;
    double lastRight = 0;

    while (closestPoint(smoothed) != smoothed.size() - 1) {
        vector<double> lookAheadPoint = lookaheadPoint(smoothed, lookaheadDistance);

        double wheelLeft = smoothed[closestPoint(smoothed)][4] * (2 + lookaheadCurvature(lookAheadPoint, lookaheadDistance) * trackWidth) / 2;
        double wheelRight = smoothed[closestPoint(smoothed)][4] * (2 - lookaheadCurvature(lookAheadPoint, lookaheadDistance) * trackWidth) / 2;

        double tal = wheelLeft - lastLeft / 0.01;
        double tar = wheelRight - lastRight / 0.01;

        double leftWheels = (kV * wheelLeft) + (kA * tal) + (kP * (wheelLeft - l1.get_actual_velocity()));
        double rightWheels = (kV * wheelRight) + (kA * tar) + (kP * (wheelRight - r1.get_actual_velocity()));

        lastLeft = wheelLeft;
        lastRight = wheelRight;

        setDrive(leftWheels, rightWheels);
        wait(10);
    }
}
