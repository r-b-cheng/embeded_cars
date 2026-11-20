#include <math.h>
#include <string.h>
#include "protocol.h"

/* The simulated floor uses independent ground truth. Only the C controller's
   estimated pose and simulated sonar are exposed to the phone. */
static Car car;
static double x, y, theta;
static uint32_t last, ping;
static unsigned scan;
static bool obstacle_course;
static int contacts;
static const int sequence[] = {FRONT, LEFT, FRONT, RIGHT};
static const int degrees[] = {0, 35, -35};
static const double pi = 3.141592653589793;
static const double boxes[][4] = {{2600, -2000, 3400, 2000},
                                  {-2200, 2000, -1200, 2700},
                                  {800, 2500, 1800, 3200},
                                  {-3000, -3100, 1000, -2850},
                                  {3800, -4200, 4050, -1200}};

static bool solid(double a, double b) {
    if (obstacle_course) {
        return a < -1400 || a > 2500 || fabs(b) > 1400 || (a >= 700 && a <= 900 && fabs(b) <= 200);
    }
    if (fabs(a) >= 6000 || fabs(b) >= 5000)
        return true;
    for (unsigned i = 0; i < sizeof boxes / sizeof *boxes; i++)
        if (a >= boxes[i][0] && a <= boxes[i][2] && b >= boxes[i][1] && b <= boxes[i][3])
            return true;
    return false;
}
void sim_reset(void) {
    obstacle_course = false;
    contacts = 0;
    memset(&car, 0, sizeof car);
    car.command = 'S';
    x = y = theta = 0;
    last = ping = scan = 0;
}
void sim_set_course(void) { obstacle_course = true; }
int sim_true_x(void) { return (int)lround(x); }
int sim_true_y(void) { return (int)lround(y); }
int sim_contacts(void) { return contacts; }
void sim_tick(uint32_t now) {
    double dt = (now - last) / 1000.0;
    double l = car.left * 2.98, r = car.right * 3.02;
    double turn = (r - l) * dt / 135, dist = (r + l) * dt / 2;
    double nx = x + dist * cos(theta + turn / 2), ny = y + dist * sin(theta + turn / 2);
    bool touch = false;
    for (int k = 0; k < 16; k++)
        if (solid(nx + 100 * cos(k * pi / 8), ny + 100 * sin(k * pi / 8)))
            touch = true;
    if (!touch) {
        x = nx;
        y = ny;
    } else if (fabs(dist) > .01)
        contacts++;
    theta += turn;
    last = now;
    car_tick(&car, now, false);
    if (now - ping >= 150) {
        int sector = sequence[scan++ % 4];
        double a = theta + degrees[sector] * pi / 180;
        int mm = 20;
        while (mm <= 4000 && !solid(x + mm * cos(a), y + mm * sin(a)))
            mm += 10;
        car_measure(&car, sector, mm <= 4000 ? mm : -1, now);
        ping = now;
    }
}
const char *sim_line(const char *input, uint32_t now) {
    static char line[REPLY_SIZE];
    Request r = {0};
    if (!parse_request(input, &r)) {
        car_command(&car, 'S', now);
        return NULL;
    }
    apply_request(&car, &r, now);
    telemetry(line, &car, r.seq);
    return line;
}
