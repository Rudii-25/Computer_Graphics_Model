#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265358979f
#define RAD(d) ((d) * PI / 180.0f)
#define STEP_TIME 4.5f
#define STEP_COUNT 12

typedef struct {
    float tx, ty, tz;
    float rx, ry, rz;
    float sx, sy, sz, su;
    float shxy, shyx;
    int fx, fy, fz;
} Pose;

typedef struct {
    float m[16];
} Mat4;

typedef struct {
    Mat4 T, R, H, S, F, M;
} PoseMats;

static const Pose POSE_HOME = {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0};
static const float LIGHT[3] = {0.45f, 1.0f, 0.6f};

static Pose pose;
static float rotorSpin = 0, tailSpin = 0, cycRoll = 0, cycPitch = 0;
static int rotorsOn = 0, wireOn = 0, ghostOn = 1, shadowOn = 1, helpOn = 1;
static int pass = 0;

static float camYaw = 40, camPitch = 22, camDist = 7.5f;
static int dragX, dragY, dragging = 0;
static int winW = 1100, winH = 700;
static GLUquadric *quad;

static int demoOn = 0, demoStep = 0;
static float demoT = 0;
static int lastTick = 0;

static const char *stepName[STEP_COUNT] = {
    "Translation",
    "Rotation about the Y axis (yaw)",
    "Rotation about the X axis (roll)",
    "Rotation about the Z axis (pitch)",
    "Uniform scaling",
    "Non-uniform scaling",
    "Shearing",
    "Reflection about the YZ plane",
    "Reflection about the XY plane",
    "Reflection about the XZ plane",
    "Hierarchical motion (rotors and cyclic tilt)",
    "Composite transformation"
};

static const char *stepFormula[STEP_COUNT] = {
    "x' = x + tx    y' = y + ty    z' = z + tz",
    "Ry(a):  x' = x cos a + z sin a    z' = -x sin a + z cos a",
    "Rx(a):  y' = y cos a - z sin a    z' = y sin a + z cos a",
    "Rz(a):  x' = x cos a - y sin a    y' = x sin a + y cos a",
    "S(k,k,k): every coordinate is multiplied by k",
    "S(sx,sy,sz): different factors change the proportions",
    "Shear: x' = x + k*y   (k sits in row 1, column 2)",
    "F = diag(-1, 1, 1):  x -> -x   (mirror in the YZ plane)",
    "F = diag(1, 1, -1):  z -> -z   (mirror in the XY plane)",
    "F = diag(1, -1, 1):  y -> -y   (mirror in the XZ plane)",
    "child matrix = parent matrix * local matrix",
    "M = F * T * Ry * Rx * Rz * H * S   (applied right to left)"
};

static Mat4 matIdentity(void)
{
    Mat4 r;
    memset(&r, 0, sizeof r);
    r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
    return r;
}

static Mat4 matMul(Mat4 a, Mat4 b)
{
    Mat4 r;
    for (int c = 0; c < 4; c++)
        for (int row = 0; row < 4; row++) {
            float sum = 0;
            for (int k = 0; k < 4; k++)
                sum += a.m[k * 4 + row] * b.m[c * 4 + k];
            r.m[c * 4 + row] = sum;
        }
    return r;
}

static Mat4 matTranslate(float x, float y, float z)
{
    Mat4 r = matIdentity();
    r.m[12] = x;
    r.m[13] = y;
    r.m[14] = z;
    return r;
}

static Mat4 matScale(float x, float y, float z)
{
    Mat4 r = matIdentity();
    r.m[0] = x;
    r.m[5] = y;
    r.m[10] = z;
    return r;
}

static Mat4 matRotX(float deg)
{
    Mat4 r = matIdentity();
    float c = cosf(RAD(deg)), s = sinf(RAD(deg));
    r.m[5] = c;
    r.m[6] = s;
    r.m[9] = -s;
    r.m[10] = c;
    return r;
}

static Mat4 matRotY(float deg)
{
    Mat4 r = matIdentity();
    float c = cosf(RAD(deg)), s = sinf(RAD(deg));
    r.m[0] = c;
    r.m[2] = -s;
    r.m[8] = s;
    r.m[10] = c;
    return r;
}

static Mat4 matRotZ(float deg)
{
    Mat4 r = matIdentity();
    float c = cosf(RAD(deg)), s = sinf(RAD(deg));
    r.m[0] = c;
    r.m[1] = s;
    r.m[4] = -s;
    r.m[5] = c;
    return r;
}

static Mat4 matShear(float xy, float yx)
{
    Mat4 r = matIdentity();
    r.m[4] = xy;
    r.m[1] = yx;
    return r;
}

static Mat4 shadowMatrix(const float l[3])
{
    Mat4 r = matIdentity();
    r.m[4] = -l[0] / l[1];
    r.m[5] = 0.0f;
    r.m[6] = -l[2] / l[1];
    r.m[13] = 0.008f;
    return r;
}

static PoseMats buildPose(const Pose *p)
{
    PoseMats pm;
    Mat4 ry = matRotY(p->ry), rx = matRotX(p->rx), rz = matRotZ(p->rz);
    pm.T = matTranslate(p->tx, p->ty, p->tz);
    pm.R = matMul(ry, matMul(rx, rz));
    pm.H = matShear(p->shxy, p->shyx);
    pm.S = matScale(p->sx * p->su, p->sy * p->su, p->sz * p->su);
    pm.F = matScale(p->fx ? -1.0f : 1.0f, p->fy ? -1.0f : 1.0f, p->fz ? -1.0f : 1.0f);
    pm.M = matMul(pm.F, matMul(pm.T, matMul(pm.R, matMul(pm.H, pm.S))));
    return pm;
}

static void printMat(const char *name, const Mat4 *a)
{
    printf("%s\n", name);
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            float v = a->m[c * 4 + r];
            printf("%9.3f", fabsf(v) < 5e-4f ? 0.0f : v);
        }
        printf("\n");
    }
    printf("\n");
}

static void printAll(void)
{
    PoseMats pm = buildPose(&pose);
    printf("======== current transformation matrices ========\n");
    printMat("Translation  T", &pm.T);
    printMat("Rotation     R = Ry*Rx*Rz", &pm.R);
    printMat("Shear        H", &pm.H);
    printMat("Scale        S", &pm.S);
    printMat("Reflection   F", &pm.F);
    printMat("Composite    M = F*T*R*H*S", &pm.M);
}

static float ease(float x)
{
    if (x < 0) x = 0;
    if (x > 1) x = 1;
    return x * x * (3.0f - 2.0f * x);
}

static float pulse(float t)
{
    return ease(fminf(1.0f, sinf(PI * t) * 1.6f));
}

static void paint(float r, float g, float b)
{
    if (pass == 0) glColor3f(r, g, b);
}

static void tube(float ax, float ay, float az, float bx, float by, float bz, float r0, float r1)
{
    float dx = bx - ax, dy = by - ay, dz = bz - az;
    float len = sqrtf(dx * dx + dy * dy + dz * dz);
    float axisX = -dy, axisY = dx;
    float axisLen = sqrtf(axisX * axisX + axisY * axisY);
    float cs;
    if (len < 1e-6f) return;
    cs = dz / len;
    if (cs > 1.0f) cs = 1.0f;
    if (cs < -1.0f) cs = -1.0f;
    glPushMatrix();
    glTranslatef(ax, ay, az);
    if (axisLen > 1e-6f) glRotatef(acosf(cs) * 180.0f / PI, axisX, axisY, 0);
    else if (dz < 0) glRotatef(180, 1, 0, 0);
    gluCylinder(quad, r0, r1, len, 14, 1);
    gluDisk(quad, 0, r0, 14, 1);
    glTranslatef(0, 0, len);
    gluDisk(quad, 0, r1, 14, 1);
    glPopMatrix();
}

static void block(float x, float y, float z, float w, float h, float d)
{
    glPushMatrix();
    glTranslatef(x, y, z);
    glScalef(w, h, d);
    glutSolidCube(1.0);
    glPopMatrix();
}

static void ellipsoid(float x, float y, float z, float rx, float ry, float rz)
{
    glPushMatrix();
    glTranslatef(x, y, z);
    glScalef(rx, ry, rz);
    glutSolidSphere(1.0, 32, 24);
    glPopMatrix();
}

static void band(float x, float half)
{
    float k = sqrtf(1.0f - x * x);
    ellipsoid(x, 0.78f, 0, half, 0.48f * k + 0.018f, 0.42f * k + 0.018f);
}

static void drawFuselage(void)
{
    int s;
    paint(0.78f, 0.09f, 0.10f);
    ellipsoid(0, 0.78f, 0, 1.0f, 0.48f, 0.42f);
    paint(0.96f, 0.96f, 0.98f);
    band(-0.45f, 0.12f);
    band(-0.72f, 0.08f);
    paint(0.45f, 0.75f, 0.92f);
    ellipsoid(0.52f, 0.90f, 0, 0.55f, 0.34f, 0.36f);
    paint(0.12f, 0.20f, 0.30f);
    for (s = -1; s <= 1; s += 2)
        ellipsoid(0.10f, 0.90f, 0.30f * s, 0.30f, 0.17f, 0.12f);
    paint(0.20f, 0.20f, 0.23f);
    ellipsoid(-0.20f, 1.28f, 0, 0.50f, 0.17f, 0.26f);
    paint(0.95f, 0.85f, 0.20f);
    ellipsoid(1.0f, 0.62f, 0, 0.06f, 0.06f, 0.06f);
    paint(0.90f, 0.10f, 0.10f);
    ellipsoid(-0.20f, 1.46f, 0, 0.045f, 0.045f, 0.045f);
    paint(0.78f, 0.80f, 0.85f);
    tube(-0.10f, 1.40f, 0, -0.10f, 1.72f, 0, 0.05f, 0.05f);
}

static void drawSkids(void)
{
    int s;
    paint(0.72f, 0.74f, 0.78f);
    for (s = -1; s <= 1; s += 2) {
        float z = 0.52f * s;
        tube(-0.85f, 0.05f, z, 0.95f, 0.05f, z, 0.032f, 0.032f);
        tube(0.95f, 0.05f, z, 1.17f, 0.24f, z, 0.032f, 0.028f);
        tube(-0.85f, 0.05f, z, -0.95f, 0.20f, z, 0.032f, 0.028f);
        tube(0.50f, 0.05f, z, 0.40f, 0.50f, 0.30f * s, 0.025f, 0.025f);
        tube(-0.40f, 0.05f, z, -0.30f, 0.52f, 0.30f * s, 0.025f, 0.025f);
    }
    tube(0.45f, 0.28f, -0.41f, 0.45f, 0.28f, 0.41f, 0.02f, 0.02f);
    tube(-0.35f, 0.285f, -0.41f, -0.35f, 0.285f, 0.41f, 0.02f, 0.02f);
}

static void drawTail(void)
{
    paint(0.78f, 0.09f, 0.10f);
    tube(-0.85f, 0.85f, 0, -2.60f, 1.00f, 0, 0.22f, 0.07f);
    paint(0.96f, 0.96f, 0.98f);
    glPushMatrix();
    glTranslatef(-2.55f, 1.28f, 0);
    glRotatef(25, 0, 0, 1);
    block(0, 0, 0, 0.40f, 0.62f, 0.05f);
    glPopMatrix();
    paint(0.78f, 0.09f, 0.10f);
    block(-2.35f, 0.98f, 0, 0.30f, 0.03f, 0.80f);
    paint(0.78f, 0.80f, 0.85f);
    tube(-1.50f, 1.06f, 0, -1.50f, 1.30f, 0, 0.01f, 0.01f);
}

static void drawTailRotor(void)
{
    glPushMatrix();
    glTranslatef(-2.62f, 1.30f, 0.10f);
    glRotatef(tailSpin, 0, 0, 1);
    paint(0.85f, 0.85f, 0.88f);
    block(0, 0, 0, 0.06f, 0.64f, 0.015f);
    block(0, 0, 0, 0.64f, 0.06f, 0.017f);
    paint(0.20f, 0.20f, 0.23f);
    ellipsoid(0, 0, 0.02f, 0.05f, 0.05f, 0.05f);
    glPopMatrix();
}

static void drawMainRotor(void)
{
    int i;
    glPushMatrix();
    glTranslatef(-0.10f, 1.72f, 0);
    glRotatef(cycRoll, 1, 0, 0);
    glRotatef(cycPitch, 0, 0, 1);
    paint(0.22f, 0.22f, 0.25f);
    tube(0, 0, 0, 0, 0.10f, 0, 0.11f, 0.09f);
    glPushMatrix();
    glRotatef(rotorSpin, 0, 1, 0);
    for (i = 0; i < 4; i++) {
        glPushMatrix();
        glRotatef(i * 90.0f, 0, 1, 0);
        paint(0.12f, 0.12f, 0.14f);
        block(1.0f, 0.06f, 0, 1.9f, 0.025f, 0.14f);
        paint(0.95f, 0.75f, 0.10f);
        block(1.82f, 0.06f, 0, 0.26f, 0.027f, 0.14f);
        glPopMatrix();
    }
    glPopMatrix();
    if (rotorsOn && pass == 0 && !wireOn) {
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        glColor4f(0.75f, 0.80f, 0.90f, 0.14f);
        glPushMatrix();
        glTranslatef(0, 0.06f, 0);
        glRotatef(-90, 1, 0, 0);
        gluDisk(quad, 0, 1.92f, 48, 1);
        glPopMatrix();
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glEnable(GL_LIGHTING);
    }
    glPopMatrix();
}

static void drawHelicopter(void)
{
    drawFuselage();
    drawSkids();
    drawTail();
    drawTailRotor();
    drawMainRotor();
}

static void drawGround(void)
{
    int i;
    glDisable(GL_LIGHTING);
    glDepthMask(GL_FALSE);
    glColor3f(0.10f, 0.15f, 0.12f);
    glBegin(GL_QUADS);
    glVertex3f(-9, -0.004f, -9);
    glVertex3f(-9, -0.004f, 9);
    glVertex3f(9, -0.004f, 9);
    glVertex3f(9, -0.004f, -9);
    glEnd();
    glColor3f(0.20f, 0.27f, 0.23f);
    glBegin(GL_LINES);
    for (i = -9; i <= 9; i++) {
        glVertex3f((float)i, 0, -9);
        glVertex3f((float)i, 0, 9);
        glVertex3f(-9, 0, (float)i);
        glVertex3f(9, 0, (float)i);
    }
    glEnd();
    glPushMatrix();
    glRotatef(-90, 1, 0, 0);
    glColor3f(0.27f, 0.29f, 0.33f);
    glTranslatef(0, 0, 0.002f);
    gluDisk(quad, 0, 2.3f, 64, 1);
    glColor3f(0.95f, 0.95f, 0.95f);
    glTranslatef(0, 0, 0.001f);
    gluDisk(quad, 2.05f, 2.2f, 64, 1);
    glPopMatrix();
    glColor3f(0.95f, 0.95f, 0.95f);
    glBegin(GL_QUADS);
    glVertex3f(-0.7f, 0.004f, -0.7f);
    glVertex3f(-0.7f, 0.004f, 0.7f);
    glVertex3f(-0.5f, 0.004f, 0.7f);
    glVertex3f(-0.5f, 0.004f, -0.7f);
    glVertex3f(0.5f, 0.004f, -0.7f);
    glVertex3f(0.5f, 0.004f, 0.7f);
    glVertex3f(0.7f, 0.004f, 0.7f);
    glVertex3f(0.7f, 0.004f, -0.7f);
    glVertex3f(-0.5f, 0.004f, -0.1f);
    glVertex3f(-0.5f, 0.004f, 0.1f);
    glVertex3f(0.5f, 0.004f, 0.1f);
    glVertex3f(0.5f, 0.004f, -0.1f);
    glEnd();
    glLineWidth(2);
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.3f, 0.3f);
    glVertex3f(0, 0.012f, 0);
    glVertex3f(2.8f, 0.012f, 0);
    glColor3f(0.3f, 1.0f, 0.4f);
    glVertex3f(0, 0.012f, 0);
    glVertex3f(0, 2.8f, 0);
    glColor3f(0.4f, 0.6f, 1.0f);
    glVertex3f(0, 0.012f, 0);
    glVertex3f(0, 0.012f, 2.8f);
    glEnd();
    glLineWidth(1);
    glDepthMask(GL_TRUE);
    glEnable(GL_LIGHTING);
}

static void drawShadow(const PoseMats *pm)
{
    Mat4 sh = shadowMatrix(LIGHT);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_EQUAL, 0, 1);
    glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
    glColor4f(0, 0, 0, 0.45f);
    pass = 2;
    glPushMatrix();
    glMultMatrixf(sh.m);
    glMultMatrixf(pm->M.m);
    drawHelicopter();
    glPopMatrix();
    pass = 0;
    glDisable(GL_STENCIL_TEST);
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
}

static void text(float x, float y, const char *s, void *font)
{
    glRasterPos2f(x, y);
    for (; *s; s++) glutBitmapCharacter(font, *s);
}

static void drawOverlay(const PoseMats *pm)
{
    char buf[200];
    float bx = winW - 340.0f, by = winH - 25.0f;
    static const char *help[10] = {
        "CONTROLS  (H = hide)",
        "SPACE  demo on/off          , .  previous / next step",
        "Arrows  move X / Z          PgUp PgDn  move Y",
        "x X   y Y   z Z   rotate about X / Y / Z",
        "+ -  uniform scale          j J   k K   l L   Sx Sy Sz",
        "u U  shear x += k*y         i I  shear y += k*x",
        "7 8 9   reflect about YZ / XZ / XY plane",
        "a d  cyclic roll    f b  cyclic pitch    c  rotors",
        "w wireframe   g ghost   p shadow   m print matrices   r reset",
        "Mouse drag = orbit     wheel or [ ] = zoom     ESC = quit"
    };

    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, winW, 0, winH);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    if (demoOn) {
        glColor3f(1, 1, 1);
        snprintf(buf, sizeof buf, "Step %d/%d: %s", demoStep + 1, STEP_COUNT, stepName[demoStep]);
        text(20, winH - 25, buf, GLUT_BITMAP_HELVETICA_18);
        glColor3f(0.6f, 0.9f, 0.9f);
        text(20, winH - 50, stepFormula[demoStep], GLUT_BITMAP_HELVETICA_18);
    }

    glColor3f(0.55f, 0.70f, 1.0f);
    text(bx, by, "Composite matrix M = F*T*R*H*S", GLUT_BITMAP_9_BY_15);
    glColor3f(0.90f, 0.95f, 1.0f);
    for (int r = 0; r < 4; r++) {
        snprintf(buf, sizeof buf, "%8.2f%8.2f%8.2f%8.2f",
                 pm->M.m[r], pm->M.m[4 + r], pm->M.m[8 + r], pm->M.m[12 + r]);
        text(bx, by - 20 - r * 17, buf, GLUT_BITMAP_9_BY_15);
    }
    glColor3f(0.70f, 0.70f, 0.75f);
    snprintf(buf, sizeof buf, "T(%.2f, %.2f, %.2f)", pose.tx, pose.ty, pose.tz);
    text(bx, by - 110, buf, GLUT_BITMAP_HELVETICA_18);
    snprintf(buf, sizeof buf, "R(x=%.0f, y=%.0f, z=%.0f) deg", pose.rx, pose.ry, pose.rz);
    text(bx, by - 134, buf, GLUT_BITMAP_HELVETICA_18);
    snprintf(buf, sizeof buf, "S(%.2f, %.2f, %.2f)", pose.sx * pose.su, pose.sy * pose.su, pose.sz * pose.su);
    text(bx, by - 158, buf, GLUT_BITMAP_HELVETICA_18);
    snprintf(buf, sizeof buf, "Shear(xy=%.2f, yx=%.2f)", pose.shxy, pose.shyx);
    text(bx, by - 182, buf, GLUT_BITMAP_HELVETICA_18);
    snprintf(buf, sizeof buf, "Reflect(x=%d, y=%d, z=%d)", pose.fx, pose.fy, pose.fz);
    text(bx, by - 206, buf, GLUT_BITMAP_HELVETICA_18);
    snprintf(buf, sizeof buf, "Cyclic(roll=%.0f, pitch=%.0f)", cycRoll, cycPitch);
    text(bx, by - 230, buf, GLUT_BITMAP_HELVETICA_18);

    if (helpOn) {
        glColor3f(0.85f, 0.88f, 0.95f);
        for (int i = 0; i < 10; i++)
            text(15, 15 + (9 - i) * 24, help[i], GLUT_BITMAP_HELVETICA_18);
    }

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
}

static void display(void)
{
    PoseMats pm = buildPose(&pose);
    GLfloat lightPos[4];
    float cp = cosf(RAD(camPitch));

    glClearColor(0.05f, 0.08f, 0.14f, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, (double)winW / winH, 0.1, 100);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(camDist * cp * sinf(RAD(camYaw)), 0.9f + camDist * sinf(RAD(camPitch)), camDist * cp * cosf(RAD(camYaw)),
              0, 0.9f, 0, 0, 1, 0);

    lightPos[0] = LIGHT[0];
    lightPos[1] = LIGHT[1];
    lightPos[2] = LIGHT[2];
    lightPos[3] = 0;
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

    drawGround();

    if (ghostOn && !wireOn) {
        glDisable(GL_LIGHTING);
        glColor3f(0.50f, 0.55f, 0.65f);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        pass = 1;
        drawHelicopter();
        pass = 0;
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glEnable(GL_LIGHTING);
    }

    if (shadowOn && !pose.fy) drawShadow(&pm);

    glPushMatrix();
    glMultMatrixf(pm.M.m);
    if (wireOn) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    drawHelicopter();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glPopMatrix();

    drawOverlay(&pm);
    glutSwapBuffers();
}

static void applyDemo(void)
{
    float t = demoT;
    float e = pulse(t);
    float m = ease(fminf(1.0f, t * 3.0f));
    int flip = t > 0.5f;
    pose = POSE_HOME;
    if (demoStep != 10) {
        cycRoll = 0;
        cycPitch = 0;
    }
    switch (demoStep) {
    case 0:
        pose.tx = 2.2f * e;
        pose.ty = 0.9f * e;
        pose.tz = 1.3f * e;
        break;
    case 1:
        pose.ry = 360.0f * ease(t);
        break;
    case 2:
        pose.rx = 35.0f * e;
        break;
    case 3:
        pose.rz = 28.0f * e;
        break;
    case 4:
        pose.su = 1.0f + 0.8f * e;
        break;
    case 5:
        pose.sx = 1.0f + 0.7f * e;
        pose.sy = 1.0f - 0.4f * e;
        pose.sz = 1.0f + 0.8f * e;
        break;
    case 6:
        pose.shxy = 0.7f * e;
        break;
    case 7:
        pose.tx = 2.2f * m;
        pose.fx = flip;
        break;
    case 8:
        pose.tz = 2.0f * m;
        pose.fz = flip;
        break;
    case 9:
        pose.ty = 1.6f * m;
        pose.fy = flip;
        break;
    case 10:
        cycRoll = 14.0f * sinf(t * 4.0f * PI);
        cycPitch = 12.0f * cosf(t * 4.0f * PI);
        break;
    case 11:
        pose.tx = 1.4f * e;
        pose.ty = 0.5f * e;
        pose.tz = -1.4f * e;
        pose.ry = 140.0f * e;
        pose.rz = 15.0f * e;
        pose.su = 1.0f + 0.2f * e;
        pose.shxy = 0.25f * e;
        break;
    }
}

static void reshape(int w, int h)
{
    winW = w;
    winH = h > 0 ? h : 1;
    glViewport(0, 0, winW, winH);
}

static void tick(int v)
{
    int now = glutGet(GLUT_ELAPSED_TIME);
    float dt = (now - lastTick) / 1000.0f;
    (void)v;
    lastTick = now;
    if (dt > 0.05f) dt = 0.05f;

    if (demoOn) {
        demoT += dt / STEP_TIME;
        if (demoT >= 1.0f) {
            demoT = 0;
            demoStep = (demoStep + 1) % STEP_COUNT;
        }
        applyDemo();
    }
    if (rotorsOn) {
        rotorSpin = fmodf(rotorSpin + 540.0f * dt, 360.0f);
        tailSpin = fmodf(tailSpin + 1300.0f * dt, 360.0f);
    }
    glutPostRedisplay();
    glutTimerFunc(16, tick, 0);
}

static void startDemo(void)
{
    demoOn = 1;
    rotorsOn = 1;
}

static void stopDemo(void)
{
    demoOn = 0;
}

static void keyboard(unsigned char k, int mx, int my)
{
    float st = 0.1f;
    (void)mx;
    (void)my;
    switch (k) {
    case 27:
        exit(0);
    case ' ':
        if (demoOn) stopDemo();
        else {
            demoT = 0;
            startDemo();
        }
        return;
    case '.':
        startDemo();
        demoStep = (demoStep + 1) % STEP_COUNT;
        demoT = 0;
        return;
    case ',':
        startDemo();
        demoStep = (demoStep + STEP_COUNT - 1) % STEP_COUNT;
        demoT = 0;
        return;
    case 'h': case 'H': helpOn = !helpOn; return;
    case 'w': case 'W': wireOn = !wireOn; return;
    case 'g': case 'G': ghostOn = !ghostOn; return;
    case 'p': case 'P': shadowOn = !shadowOn; return;
    case 'c': case 'C': rotorsOn = !rotorsOn; return;
    case 'm': case 'M': printAll(); return;
    case '[': camDist *= 0.92f; return;
    case ']': camDist *= 1.08f; return;
    case 'a': cycRoll -= 2; if (cycRoll < -20) cycRoll = -20; return;
    case 'd': cycRoll += 2; if (cycRoll > 20) cycRoll = 20; return;
    case 'f': cycPitch -= 2; if (cycPitch < -20) cycPitch = -20; return;
    case 'b': cycPitch += 2; if (cycPitch > 20) cycPitch = 20; return;
    case 'r': case 'R':
        stopDemo();
        pose = POSE_HOME;
        cycRoll = cycPitch = rotorSpin = tailSpin = 0;
        rotorsOn = 0;
        return;
    }
    stopDemo();
    switch (k) {
    case 'x': pose.rx += 5; break;
    case 'X': pose.rx -= 5; break;
    case 'y': pose.ry += 5; break;
    case 'Y': pose.ry -= 5; break;
    case 'z': pose.rz += 5; break;
    case 'Z': pose.rz -= 5; break;
    case '+': case '=': pose.su *= 1.1f; break;
    case '-': case '_': pose.su /= 1.1f; break;
    case 'j': pose.sx *= 1.1f; break;
    case 'J': pose.sx /= 1.1f; break;
    case 'k': pose.sy *= 1.1f; break;
    case 'K': pose.sy /= 1.1f; break;
    case 'l': pose.sz *= 1.1f; break;
    case 'L': pose.sz /= 1.1f; break;
    case 'u': pose.shxy += st; break;
    case 'U': pose.shxy -= st; break;
    case 'i': pose.shyx += st; break;
    case 'I': pose.shyx -= st; break;
    case '7': pose.fx = !pose.fx; break;
    case '8': pose.fy = !pose.fy; break;
    case '9': pose.fz = !pose.fz; break;
    }
}

static void special(int k, int mx, int my)
{
    float st = 0.15f;
    (void)mx;
    (void)my;
    stopDemo();
    switch (k) {
    case GLUT_KEY_LEFT: pose.tx -= st; break;
    case GLUT_KEY_RIGHT: pose.tx += st; break;
    case GLUT_KEY_UP: pose.tz -= st; break;
    case GLUT_KEY_DOWN: pose.tz += st; break;
    case GLUT_KEY_PAGE_UP: pose.ty += st; break;
    case GLUT_KEY_PAGE_DOWN: pose.ty -= st; break;
    }
}

static void mouse(int button, int state, int x, int y)
{
    if (button == GLUT_LEFT_BUTTON) {
        dragging = (state == GLUT_DOWN);
        dragX = x;
        dragY = y;
    }
    if (state == GLUT_DOWN && button == 3) camDist *= 0.92f;
    if (state == GLUT_DOWN && button == 4) camDist *= 1.08f;
    if (camDist < 3) camDist = 3;
    if (camDist > 22) camDist = 22;
}

static void motion(int x, int y)
{
    if (!dragging) return;
    camYaw -= (x - dragX) * 0.5f;
    camPitch += (y - dragY) * 0.5f;
    if (camPitch < -10) camPitch = -10;
    if (camPitch > 85) camPitch = 85;
    dragX = x;
    dragY = y;
}

int main(int argc, char **argv)
{
    GLfloat amb[4] = {0.30f, 0.30f, 0.30f, 1};
    GLfloat dif[4] = {0.90f, 0.90f, 0.90f, 1};
    GLfloat spc[4] = {0.45f, 0.45f, 0.45f, 1};

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STENCIL);
    glutInitWindowSize(winW, winH);
    glutCreateWindow("3D Helicopter - Geometric Transformations");

    pose = POSE_HOME;
    quad = gluNewQuadric();

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_NORMALIZE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    glLightfv(GL_LIGHT0, GL_AMBIENT, amb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, dif);
    glLightfv(GL_LIGHT0, GL_SPECULAR, spc);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spc);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 40.0f);
    glShadeModel(GL_SMOOTH);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(special);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    lastTick = glutGet(GLUT_ELAPSED_TIME);
    glutTimerFunc(16, tick, 0);

    printf("3D Helicopter - press SPACE for the automatic demo, H to toggle the help text.\n");
    glutMainLoop();
    return 0;
}
