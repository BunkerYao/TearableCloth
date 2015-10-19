// ����ģ����� Դ����
// ����һ��ʹ��Verlet���ַ�ʵ�ֵĶ�ά��������ģ�����
// ͼ�λ���ʹ����Direct2D���������ֻ���ʹ����DirectWrite����

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <tchar.h>

#include "targetver.h"
#include <windows.h>
#include <windowsx.h>
#include <d2d1.h>
#include <d2d1helper.h>
#include <dwrite.h>

// ���ڰ�ȫ�ͷ�COM�ӿڵĺ�
#define SAFE_RELEASE(p) { if (p) { p->Release(); p = nullptr; } }

// �ʵ�ṹ��
struct Point
{
    float   x, y;
    float   oldx, oldy;
    int     is_pinned;
    float   pinx, piny;
};

// С���ṹ��
struct Stick
{
    Point   *p0, *p1;
    float   length;
    bool    is_breaked;
};

//
// ȫ�ֳ�������
//

const int               SCREEN_WIDTH        = 800;                          // ��Ļ���
const int               SCREEN_HEIGHT       = 600;                          // ��Ļ�߶�
const D2D1::ColorF      BACKGROUND_COLOR    = D2D1::ColorF::Black;          // ������ɫ
const D2D1::ColorF      STICK_COLOR         = D2D1::ColorF::White;          // С����ɫ
const DWORD             UPDATE_INTERVAL     = 16;                            // ����ʱ�����ڣ����룩

//
// ȫ�ֱ�������
//

float                   g_gravity = 0.07f;                                  // ������С
float                   g_drag = 1.0f;                                      // ������������
float                   g_bouncing = 0.3f;                                  // �����ٶ�����
int                     g_iteration_count = 4;                              // ÿ֡��С�������������ĵ�������
int                     g_cloth_width = 60;                                 // ����ˮƽ������ʵ���Ŀ
int                     g_cloth_height = 40;                                // ���ϴ�ֱ������ʵ���Ŀ
float                   g_cloth_distance = 8.0f;                            // ����ÿ��С������Ȼ����
float                   g_tear_distance = 32.0f;                            // ˺��С����Ҫ�ĳ���
int                     g_point_count = g_cloth_width * g_cloth_height;                     // �ʵ�����
int                     g_stick_count = g_point_count * 2 - g_cloth_width - g_cloth_height; // С������

int                     g_mouse_x, g_mouse_y, g_mouse_oldx, g_mouse_oldy;   // ��һ֡���λ�ú͵�ǰ���λ��
int                     g_is_lbtn_down = 0;                                 // �������Ƿ��µı��
int                     g_is_rbtn_down = 0;                                 // ����Ҽ��Ƿ��µı��
int                     g_is_mbtn_down = 0;                                 // ����м��Ƿ��µı��
float                   g_mouse_influence = 30.0f;                          // ��קӰ��뾶
float                   g_mouse_cut_range = 3.5f;                           // �и�뾶
float                   g_explode_range = 25.0f;                            // ��ը�뾶

Point                   *g_points = nullptr;                                // �ʵ�����
Stick                   *g_sticks = nullptr;                                // С������

HWND                    g_hWnd = NULL;                                      // ���ھ��
ID2D1Factory            *g_d2d_factory = nullptr;                           
ID2D1HwndRenderTarget   *g_d2d_rt = nullptr;
ID2D1SolidColorBrush    *g_d2d_brush = nullptr;
IDWriteFactory          *g_dwrite_factory = nullptr;

// �������뺯��
inline int round2Int(float fnum)
{
    return (int)(fnum + 0.5f);
}

// ���ݹ��ɶ�������������
inline float calcDistance(const Point *p1, const Point *p2)
{
    float dx = p2->x - p1->x;
    float dy = p2->y - p1->y;
    return sqrtf(dx * dx + dy * dy);
}

// �����������ʵ�����Ͳ������ʵ����С�����飬�Ӷ��������鲼��
void generateCloth()
{
    // ���㲼�ϳ��ȺͿ�ȣ����ص�λ��
    float cloth_width = g_cloth_width * g_cloth_distance;
    float cloth_height = g_cloth_height * g_cloth_distance;
    // Ϊ�˽����Ϸ��ڴ������룬���㲼�����Ͻ��ʵ��λ��
    float start_x = (SCREEN_WIDTH - cloth_width) * 0.5f;
    float start_y = (SCREEN_HEIGHT - cloth_height) * 0.5f;

    for (int y = 0; y < g_cloth_height; ++y) {
        for (int x = 0; x < g_cloth_width; ++x) {
            Point *p_point = g_points + y * g_cloth_width + x;
            p_point->x = start_x + x * g_cloth_distance;
            p_point->y = start_y + y * g_cloth_distance;
            p_point->oldx = p_point->x;
            p_point->oldy = p_point->y;
            p_point->is_pinned = (y == 0) ? 1 : 0;
            p_point->pinx = p_point->x;
            p_point->piny = p_point->y;
        }
    }

    int stick_count = 0;
    for (int y = 0; y < g_cloth_height; ++y) {
        for (int x = 0; x < g_cloth_width; ++x) {
            Point *p_this = g_points + y * g_cloth_width + x;
            if (x > 0) {
                Point *p_left = g_points + y * g_cloth_width + x - 1;
                Stick *p_stick = g_sticks + stick_count;
                p_stick->p0 = p_left;
                p_stick->p1 = p_this;
                p_stick->length = calcDistance(p_this, p_left);
                p_stick->is_breaked = 0;
                stick_count++;
            }
            if (y > 0) {
                Point *p_up = g_points + (y - 1) * g_cloth_width + x;
                Stick *p_stick = g_sticks + stick_count;
                p_stick->p0 = p_up;
                p_stick->p1 = p_this;
                p_stick->length = calcDistance(p_this, p_up);
                p_stick->is_breaked = 0;
                stick_count++;
            }
        }
    }
}

void updatePoints()
{
    for (int ipoints = 0; ipoints < g_point_count; ++ipoints) {
        Point *p_point = g_points + ipoints;

        if (p_point->is_pinned) {
            p_point->x = p_point->pinx;
            p_point->y = p_point->piny;
            continue;
        }

        if (g_is_lbtn_down || g_is_mbtn_down) {
            float dx = p_point->x - g_mouse_x;
            float dy = p_point->y - g_mouse_y;
            float distance = sqrtf(dx * dx + dy * dy);
            if (g_is_lbtn_down && distance < g_mouse_influence) {
                p_point->oldx = p_point->x - (g_mouse_x - g_mouse_oldx) * 0.5f;
                p_point->oldy = p_point->y - (g_mouse_y - g_mouse_oldy) * 0.5f;
            }
            if (g_is_mbtn_down && distance < g_explode_range) {
                p_point->oldx = p_point->x - dx * 2.0f;
                p_point->oldy = p_point->y - dy * 2.0f;
            }
        }

        float vx = (p_point->x - p_point->oldx) * g_drag;
        float vy = (p_point->y - p_point->oldy) * g_drag;
        p_point->oldx = p_point->x;
        p_point->oldy = p_point->y;
        p_point->x += vx;
        p_point->y += vy;

        p_point->y += g_gravity;

        if (p_point->x > SCREEN_WIDTH) {
            p_point->x = (float)SCREEN_WIDTH;
            p_point->oldx = p_point->x + vx * g_bouncing;
        }
        else if (p_point->x < 0) {
            p_point->x = 0;
            p_point->oldx = p_point->x + vx * g_bouncing;
        }
        if (p_point->y > SCREEN_HEIGHT) {
            p_point->y = (float)SCREEN_HEIGHT;
            p_point->oldy = p_point->y + vy * g_bouncing;
        }
        else if (p_point->y < 0) {
            p_point->y = 0;
            p_point->oldy = p_point->y + vy * g_bouncing;
        }
    }
}

void updateSticks()
{
    for (int iiteration = 0; iiteration < g_iteration_count; ++iiteration) {
        for (int istick = 0; istick < g_stick_count; ++istick) {
            Stick *p_stick = g_sticks + istick;

            if (p_stick->is_breaked) continue;

            if (g_is_rbtn_down) {
                float middlex = p_stick->p0->x + (p_stick->p1->x - p_stick->p0->x) * 0.5f;
                float middley = p_stick->p0->y + (p_stick->p1->y - p_stick->p0->y) * 0.5f;
                float dx = middlex - g_mouse_x;
                float dy = middley - g_mouse_y;
                float distance = sqrtf(dx * dx + dy * dy);
                if (distance < g_mouse_cut_range) {
                    p_stick->is_breaked = 1;
                }
            }

            float dx = p_stick->p1->x - p_stick->p0->x;
            float dy = p_stick->p1->y - p_stick->p0->y;
            float distance = sqrtf(dx * dx + dy * dy);
            
            if (distance > g_tear_distance)
                p_stick->is_breaked = 1;
            
            float difference = p_stick->length - distance;
            float percent = difference / distance * 0.5f;
            float offset_x = dx * percent;
            float offset_y = dy * percent;

            if (!p_stick->p0->is_pinned) {
                p_stick->p0->x -= offset_x;
                p_stick->p0->y -= offset_y;
            }
            if (!p_stick->p1->is_pinned) {
                p_stick->p1->x += offset_x;
                p_stick->p1->y += offset_y;
            }
        }
    }
}

void renderSticks()
{
    for (int istick = 0; istick < g_stick_count; ++istick) {
        const Stick *p_stick = g_sticks + istick;
        if (p_stick->is_breaked) continue;

        g_d2d_rt->DrawLine(
            D2D1::Point2F(p_stick->p0->x, p_stick->p0->y),
            D2D1::Point2F(p_stick->p1->x, p_stick->p1->y),
            g_d2d_brush,
            1.0f
            );
    }
}

void renderText(float fps)
{
    IDWriteTextFormat *format = nullptr;
    if ( FAILED ( g_dwrite_factory->CreateTextFormat(
        L"Arial",
        NULL,
        DWRITE_FONT_WEIGHT_NORMAL,
        DWRITE_FONT_STYLE_NORMAL,
        DWRITE_FONT_STRETCH_NORMAL,
        17,
        L"",
        &format
        ) ) )
    {
        return;
    }

    wchar_t charbuff[256] = { '\0' };
    swprintf_s(charbuff, L"FPS: %.2f", fps);
    g_d2d_rt->DrawTextW(charbuff, wcslen(charbuff) + 1, format, D2D1::RectF(10.0f, 5.0f, 200.0f, 50.0f), g_d2d_brush);

    wcscpy_s(charbuff, L"���ã�R\n��ק��������\n�и����Ҽ�\n���ƣ�����м�\n�˳���ESC");
    g_d2d_rt->DrawTextW(charbuff, wcslen(charbuff) + 1, format, D2D1::RectF(10.0f, 55.0f, 200.0f, 50.0f), g_d2d_brush);

    SAFE_RELEASE(format);
}

void clearScreen()
{
    g_d2d_rt->Clear(BACKGROUND_COLOR);
}

void doFrameProcess()
{
    static int last_time = 0;
    static int frame_count = 0;
    static float fps = 0.0f;
    int current_time = timeGetTime();
    int elapsed_time = current_time - last_time;
    last_time = current_time;
    int sleep_time = UPDATE_INTERVAL - elapsed_time;
    if (sleep_time > 0) Sleep(sleep_time);
    else sleep_time = 0;

    updateSticks();
    updatePoints();

    g_d2d_rt->BeginDraw();
    g_d2d_rt->SetTransform(D2D1::Matrix3x2F::Identity());

    clearScreen();
    renderSticks();

    frame_count++;

    if (frame_count == 30) {
        fps = 1000.0f / (elapsed_time + sleep_time);
        frame_count = 0;
    }

    renderText(fps);

    g_d2d_rt->EndDraw();
}

bool initApp()
{
    HRESULT hr = S_OK;
    hr = D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &g_d2d_factory);
    if (FAILED(hr)) {
        MessageBox(NULL, L"����", L"D2D1CreateFactoryʧ��", 0);
        return false;
    }
    D2D1_SIZE_U size = D2D1::SizeU(SCREEN_WIDTH, SCREEN_HEIGHT);
    hr = g_d2d_factory->CreateHwndRenderTarget(
        D2D1::RenderTargetProperties(),
        D2D1::HwndRenderTargetProperties(g_hWnd, size),
        &g_d2d_rt
        );
    if (FAILED(hr)) {
        MessageBox(NULL, L"����", L"CreateHwndRenderTargetʧ��", 0);
        return false;
    }
    hr = g_d2d_rt->CreateSolidColorBrush(STICK_COLOR, &g_d2d_brush);
    if (FAILED(hr)) {
        MessageBox(NULL, L"����", L"CreateSolidColorBrushʧ��", 0);
        return false;
    }
    hr = DWriteCreateFactory(
        DWRITE_FACTORY_TYPE_SHARED,
        __uuidof(g_dwrite_factory),
        (IUnknown**)&g_dwrite_factory
        );
    if (FAILED(hr)) {
        MessageBox(NULL, L"����", L"DWriteCreateFactoryʧ��", 0);
        return false;
    }

    g_points = (Point*)malloc(sizeof(Point) * g_point_count);
    g_sticks = (Stick*)malloc(sizeof(Stick) * g_stick_count);

    generateCloth();

    return true;
}

void finiApp()
{
    SAFE_RELEASE(g_d2d_factory);
    SAFE_RELEASE(g_d2d_rt);
    SAFE_RELEASE(g_d2d_brush);
    SAFE_RELEASE(g_dwrite_factory);

    free(g_points);
    free(g_sticks);
}

LRESULT CALLBACK WndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    switch (uMsg)
    {
    case WM_DESTROY:
        ::PostQuitMessage(0);
        return 0;

    case WM_LBUTTONDOWN:
        g_is_lbtn_down = 1;
        g_mouse_x = GET_X_LPARAM(lParam);
        g_mouse_y = GET_Y_LPARAM(lParam);
        g_mouse_oldx = g_mouse_x;
        g_mouse_oldy = g_mouse_y;
        return 0;

    case WM_LBUTTONUP:
        g_is_lbtn_down = 0;
        return 0;

    case WM_MOUSEMOVE:
        g_mouse_oldx = g_mouse_x;
        g_mouse_oldy = g_mouse_y;
        g_mouse_x = GET_X_LPARAM(lParam);
        g_mouse_y = GET_Y_LPARAM(lParam);
        return 0;

    case WM_RBUTTONDOWN:
        g_is_rbtn_down = 1;
        return 0;

    case WM_RBUTTONUP:
        g_is_rbtn_down = 0;
        return 0;

    case WM_MBUTTONDOWN:
        g_is_mbtn_down = 1;
        return 0;

    case WM_MBUTTONUP:
        g_is_mbtn_down = 0;
        return 0;

    case WM_KEYDOWN:
        if (wParam == VK_ESCAPE) {
            DestroyWindow(hWnd);
        }
        if (wParam == 'R') {
            generateCloth();
        }
        return 0;

    default:
        return ::DefWindowProcW(hWnd, uMsg, wParam, lParam);
    };
}

HWND createAppWindow(HINSTANCE hInstance)
{
    WNDCLASSEXW wcex;
    wcex.cbSize = sizeof(WNDCLASSEXW);
    wcex.style			= CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc	= WndProc;
    wcex.cbClsExtra		= 0;
    wcex.cbWndExtra		= 0;
    wcex.hInstance		= hInstance;
    wcex.hIcon			= LoadIcon(0, IDI_APPLICATION);
    wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
    wcex.hbrBackground	= 0;
    wcex.lpszMenuName	= 0;
    wcex.lpszClassName	= L"Window";
    wcex.hIconSm		= LoadIcon(0, IDI_APPLICATION);

    if (!::RegisterClassExW(&wcex)) return false;

    const DWORD WINDOW_STYLE = WS_OVERLAPPEDWINDOW & (~WS_MAXIMIZEBOX) & (~WS_SIZEBOX);

    RECT rect;
    rect.left = 100;
    rect.top = 100;
    rect.right = rect.left + SCREEN_WIDTH;
    rect.bottom = rect.top + SCREEN_HEIGHT;
    ::AdjustWindowRect(&rect, WINDOW_STYLE, FALSE); 

    HWND hWnd = ::CreateWindowW(
        L"Window", 
        L"����ģ��",
        WINDOW_STYLE,
        rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top,
        NULL, 
        NULL,
        hInstance,
        NULL
        );

    if (hWnd == NULL) return NULL;

    ::ShowWindow(hWnd, SW_SHOWNORMAL);
    ::UpdateWindow(hWnd);

    return hWnd;
}

int APIENTRY _tWinMain(_In_ HINSTANCE hInstance,
                       _In_opt_ HINSTANCE hPrevInstance,
                       _In_ LPTSTR    lpCmdLine,
                       _In_ int       nCmdShow)
{
    g_hWnd = createAppWindow(hInstance);

    if ( initApp() ) {
        MSG msg;
        while (true) {
            if ( ::PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE) ) {
                if (msg.message == WM_QUIT) 
                    break;

                ::TranslateMessage(&msg);
                ::DispatchMessageW(&msg);
            }
            else {
                doFrameProcess();
            }
        }
    }

    finiApp();

    return 0;
}

