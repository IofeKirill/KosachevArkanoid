#include <opencv2/opencv.hpp> // подключение библиотеки opencv
#include <vector>
#include <winsock2.h> // основные сетевые типы и функции (сокеты)
#include <ws2tcpip.h> // удобные функции для IP-адресов и портов
#pragma comment(lib, "ws2_32.lib") // библиотека линковки WinSock (чтобы всё собралось).

// Глобальные переменные сокета и адреса получателя
SOCKET senderSocket = INVALID_SOCKET;  // сокет для отправки
sockaddr_in receiverAddr{};            // структура с IP и портом получателя

struct MinMax // структура для переменных
{
    int min;
    int max;
};

int S_Robot_min = 148, V_Robot_min = 103;
MinMax H_Robot = { 96, 117 };
MinMax A_Robot = { 110, 120 };
MinMax B_Robot = { 95, 120 };

int S_Ball_min = 100, V_Ball_min = 80;
MinMax H1_Ball = { 0, 15 };
MinMax H2_Ball = { 165, 179 };
MinMax A_Ball = { 150, 200 };
MinMax B_Ball = { 130, 200 };

struct Packet
{
    uint16_t x;  // координата X
    uint16_t y;  // координата Y
};

// КАЛИБРОВКА ПОЛЯ
std::vector<cv::Point2f> fieldCorners;
const int FIELD_W = 800;
const int FIELD_H = 400;
bool calibrated = false;

void onMouse(int event, int x, int y, int flags, void*)
{
    if (event == cv::EVENT_LBUTTONDOWN && fieldCorners.size() < 4)
        fieldCorners.emplace_back((float)x, (float)y);
}

static inline void toBGR(const cv::Mat& gray, cv::Mat& bgr) // если изображение одноканальное (серое) — переводит в BGR, иначе копирует как есть
{
    if (gray.empty())
    {
        bgr.release();
        return;
    }
    if (gray.channels() == 1) cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
    else bgr = gray.clone();
}

static inline cv::Mat makeGrid(const cv::Mat& upL, const cv::Mat& upR, const cv::Mat& downL, const cv::Mat& downR, int outW, int outH) // собирает 4 изображения в одну сетку 2x2 для отладочного отображения
{
    int cellW = outW / 2;
    int cellH = outH / 2;

    cv::Mat upRow, downRow, grid;
    cv::hconcat(upL, upR, upRow);
    cv::hconcat(downL, downR, downRow);
    cv::vconcat(upRow, downRow, grid);
    return grid;
}


void initRobotControls() // функция для создания трекбаров
{
    cv::namedWindow("Controls_Robot", cv::WINDOW_NORMAL);
    cv::resizeWindow("Controls_Robot", 350, 260);

    cv::createTrackbar("H min", "Controls_Robot", &H_Robot.min, 179); // создаем трекбары
    cv::createTrackbar("H max", "Controls_Robot", &H_Robot.max, 179);
    cv::createTrackbar("S min", "Controls_Robot", &S_Robot_min, 255);
    cv::createTrackbar("V min", "Controls_Robot", &V_Robot_min, 255);

    cv::createTrackbar("A min", "Controls_Robot", &A_Robot.min, 255);
    cv::createTrackbar("A max", "Controls_Robot", &A_Robot.max, 255);
    cv::createTrackbar("B min", "Controls_Robot", &B_Robot.min, 255);
    cv::createTrackbar("B max", "Controls_Robot", &B_Robot.max, 255);
}

void initBallControls() // функция для создания трекбаров
{
    cv::namedWindow("Controls_Ball", cv::WINDOW_NORMAL);
    cv::resizeWindow("Controls_Ball", 350, 260);

    cv::createTrackbar("H1 min", "Controls_Ball", &H1_Ball.min, 179);
    cv::createTrackbar("H1 max", "Controls_Ball", &H1_Ball.max, 179);
    cv::createTrackbar("H2 min", "Controls_Ball", &H2_Ball.min, 179);
    cv::createTrackbar("H2 max", "Controls_Ball", &H2_Ball.max, 179);
    cv::createTrackbar("S min", "Controls_Ball", &S_Ball_min, 255);
    cv::createTrackbar("V min", "Controls_Ball", &V_Ball_min, 255);

    cv::createTrackbar("A min", "Controls_Ball", &A_Ball.min, 255);
    cv::createTrackbar("A max", "Controls_Ball", &A_Ball.max, 255);
    cv::createTrackbar("B min", "Controls_Ball", &B_Ball.min, 255);
    cv::createTrackbar("B max", "Controls_Ball", &B_Ball.max, 255);
}

bool initUDP() // инициализация UDP-соединения
{
    WSADATA wsa; // структура для запуска WinSock
    // запускаем WinSock версии 2.2
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
        return false; // если ошибка — выходим

    // создаём UDP-сокет (IPv4, датаграммы, протокол UDP)
    senderSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    /*
    * 1 параметр - тип адреса:
    * AF_INET = IPv4.
    * AF_INET6 → IPv6
    *
    * 2 параметр - тип сокета:
    * SOCK_DGRAM = датаграммы → это UDP.
    * SOCK_STREAM → TCP (соединение, подтверждения, контроль доставки)
    *
    * 3 параметр - Какой именно протокол использовать.
    * IPPROTO_UDP → UDP
    * IPPROTO_TCP → TCP
    */

    // проверяем, создан ли сокет
    if (senderSocket == INVALID_SOCKET)
        return false;

    receiverAddr.sin_family = AF_INET;      // используем IPv4
    receiverAddr.sin_port = htons(239);     // порт получателя (перевод в сетевой формат)

    // преобразуем строковый IP в бинарный формат
    inet_pton(AF_INET, "192.168.4.1", &receiverAddr.sin_addr);

    return true; // всё успешно
}

void shutdownUDP() // корректное завершение работы UDP
{
    // если сокет открыт — закрываем его
    if (senderSocket != INVALID_SOCKET)
        closesocket(senderSocket);

    senderSocket = INVALID_SOCKET; // помечаем как закрытый

    WSACleanup(); // освобождаем ресурсы WinSock
}

void sendData(const void* data, size_t size)
{
    sendto(
        senderSocket,                              // сокет
        reinterpret_cast<const char*>(data),       // указатель на данные
        static_cast<int>(size),                    // размер данных
        0,                                         // флаги
        reinterpret_cast<sockaddr*>(&receiverAddr),// адрес получателя
        sizeof(receiverAddr)                       // размер структуры адреса
    );
}

enum class Mode // перечисление режимов работы программы
{
    ARKANOID = 1,
    ROBOT_CAL = 2,
    BALL_CAL = 3
};

Mode mode = Mode::ARKANOID;
Mode prevMode = Mode::ARKANOID;
bool robotControlsOpen = false; // открыто ли окно калибровки робота
bool ballControlsOpen = false;

int main()
{
    cv::Mat frameBGR, frameHSV, frameLAB, frameLabHSV, homography, frameTop; // объект класса Mat для хранения текущего кадра изображения
    cv::Mat frameBGRBall, frameHSVBall1, frameHSVBall2, frameHSVBall, frameLABBall, frameLabHSVBall;
    cv::VideoCapture video(0, cv::CAP_DSHOW); // создаем объект класса VideoCapture: 0 — индекс камеры, CAP_DSHOW — backend DirectShow
    if (!video.isOpened()) return -1; // если камера не открылась — завершаем программу с кодом ошибки -1

    cv::namedWindow("View", cv::WINDOW_NORMAL);
    cv::setMouseCallback("View", onMouse);

    if (!initUDP()) return -2; // инициализируем UDP
    while (true) // бесконечно
    {
        video >> frameBGR; // получаем кадр из видеопотока
        if (frameBGR.empty()) break; // если кадр пустой — выходим из цикла

        if (!calibrated)
        {
            for (int n = 0; n < fieldCorners.size(); n++)
                cv::circle(frameBGR, fieldCorners[n], 6, cv::Scalar(0, 0, 255), -1);
            int key = cv::waitKey(1);
            cv::imshow("View", frameBGR);
            if (key == 13 && fieldCorners.size() == 4)
            {
                std::vector<cv::Point2f> dst = {
                    {0, 0},
                    {(float)FIELD_W, 0},
                    {(float)FIELD_W, (float)FIELD_H},
                    {0, (float)FIELD_H}
                };
                homography = cv::getPerspectiveTransform(fieldCorners, dst);
                calibrated = true;
                cv::destroyWindow("View");
            }

            if (cv::waitKey(1) == 27) break;
            continue;
        }

        int k = cv::waitKey(1); // читаем нажатую клавишу (если есть)
        if (k == 27) break; // ESC — выход из программы
        if (k == '1') mode = Mode::ARKANOID;
        if (k == '2') mode = Mode::ROBOT_CAL;
        if (k == '3') mode = Mode::BALL_CAL;

        if (mode != prevMode)
        {
            if (ballControlsOpen) // если было открыто окно калибровки мяча — закрываем его
            {
                cv::destroyWindow("Controls_Ball");
                ballControlsOpen = false;
            }
            if (robotControlsOpen) // если было открыто окно калибровки робота — закрываем его
            {
                cv::destroyWindow("Controls_Robot");

                robotControlsOpen = false;
            }
            if (mode == Mode::ROBOT_CAL) // если режим - калибровка робота
            {
                initRobotControls(); // создаём окно и трекбары для калибровки робота
                robotControlsOpen = true;// отмечаем, что окно калибровки робота открыто
            }
            if (mode == Mode::BALL_CAL) // если режим - калибровка робота
            {
                initBallControls(); // создаём окно и трекбары для калибровки робота
                ballControlsOpen = true;// отмечаем, что окно калибровки робота открыто
            }
            prevMode = mode; // запоминаем текущий режим как предыдущий
        }

        cv::warpPerspective(frameBGR, frameTop, homography, cv::Size(FIELD_W, FIELD_H));

        //////// РОБОТ
        cv::cvtColor(frameTop, frameHSV, cv::COLOR_BGR2HSV); // преобразуем полученный кадр в HSV
        cv::GaussianBlur(frameHSV,
            frameHSV,
            cv::Size(15, 15),
            0); // размываем кадр и переписываем в ту же переменную
        cv::inRange(frameHSV,
            cv::Scalar(H_Robot.min, S_Robot_min, V_Robot_min),
            cv::Scalar(H_Robot.max, 255, 255),
            frameHSV); // ищем только вхождения нужного цвета (преобразуем в бинарную маску) и переписываем в ту же переменную

        cv::cvtColor(frameTop, frameLAB, cv::COLOR_BGR2Lab);// преобразуем полученный кадр в Lab
        std::vector<cv::Mat> vectorFrameLab; // создаем массив типа Mat (вектор - динамический массив). Нужны библиотеки vector и iostream
        cv::Mat maskA, maskB; // создаем маски для осей А и В
        cv::split(frameLAB, vectorFrameLab); // разделяем кадр в Lab на потоки
        cv::inRange(vectorFrameLab[1], A_Robot.min, A_Robot.max, maskA); // проверяем вхождения в поток А
        cv::inRange(vectorFrameLab[2], B_Robot.min, B_Robot.max, maskB); // проверяем вхождения в поток И
        frameLAB = maskA | maskB; // объединяем результат в единую маску и записываем его

        frameLabHSV = frameHSV & frameLAB; // объединяем маски HSV и Lab в единый кадр
        cv::morphologyEx(frameLabHSV, frameLabHSV, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 2); // чистим раз
        cv::morphologyEx(frameLabHSV, frameLabHSV, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 8); // чистим два
        cv::morphologyEx(frameLAB, frameLAB, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 2); // чистим раз
        cv::morphologyEx(frameLAB, frameLAB, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 8); // чистим два

        std::vector<std::vector<cv::Point>> contour; // создаем вектор векторов координат для контуров объекта. Вектор векторов - это двухмерный массив.
        cv::findContours( //Ищем контуры объекта
            frameLabHSV, // Откуда читаем
            contour, // куда выводим
            cv::RETR_EXTERNAL, // режим извлечения контуров. Берёт только внешние контуры. Варианты: RETR_LIST, RETR_TREE, RETR_CCOMP
            cv::CHAIN_APPROX_SIMPLE); // метод хранения точек контура. Оставляет только углы и ключевые точки. Вариант: CHAIN_APPROX_NONE (хранит каждый пиксель)

        double maxArea = 0; // переменная для хранения максимального контура
        int maxIndex = -1; // номер максимального контура

        for (int n = 0; n < contour.size(); n++) // пока не перебрали все контуры
        {
            double area = cv::contourArea(contour[n]); // пишем текущий контур
            if (area > maxArea) // если он больше максимального
            {
                maxArea = area; // обновляем максимальный
                maxIndex = n; // запоминаем индекс
            }
        }

        if (maxIndex != -1 && maxArea > 200) // если объект обнаружен и больше 200, то ищем его центр
        {
            cv::Point2f objectCenter; // создаем переменную для записи координат
            Packet pack; // создаем пакет для формирования данных для отправки
            float objectRadius = 0; // создаем переменную для радиуса круга
            cv::minEnclosingCircle(contour[maxIndex], objectCenter, objectRadius); // рассчитываем минимальную описанную окружность для найденного контура объекта
            cv::circle(frameTop, objectCenter, (int)objectRadius, cv::Scalar(255, 0, 0), 2); // рисуем саму окружность на изначальном изображении
            cv::circle(frameTop, objectCenter, 3, cv::Scalar(0, 255, 0), -1); // рисуем центральную точку на изначальном изображении
            cv::putText(frameTop, // добавляем надпись. что это робот
                "Robot",
                objectCenter + cv::Point2f(-objectRadius, objectRadius),
                cv::FONT_HERSHEY_COMPLEX,
                0.6,
                cv::Scalar(0, 0, 255));
            int xi = (int)std::lround(objectCenter.x); // округляем и записываем координату центра по Х 
            int yi = (int)std::lround(objectCenter.y); // округляем и записываем координату центра по У 

            xi = std::clamp(xi, 0, 65535); // ограничиваем диапазон числа, чтобы он не мог быть больше 2-х байтов
            yi = std::clamp(yi, 0, 65535);

            pack.x = htons((uint16_t)xi); // упаковываем число. преобразовав его в 2 байта
            pack.y = htons((uint16_t)yi); // упаковываем число. преобразовав его в 2 байта

            sendData(&pack, sizeof(pack)); // отправляем число по UDP при помощи созданной функции
        }

        //////// МЯЧ
        cv::cvtColor(frameTop, frameHSVBall, cv::COLOR_BGR2HSV); // преобразуем полученный кадр в HSV
        cv::GaussianBlur(frameHSVBall,
            frameHSVBall,
            cv::Size(15, 15),
            0); // размываем кадр и переписываем в ту же переменную
        cv::inRange(frameHSVBall,
            cv::Scalar(H1_Ball.min, S_Ball_min, V_Ball_min),
            cv::Scalar(H1_Ball.max, 255, 255),
            frameHSVBall1); // ищем только вхождения нужного цвета (преобразуем в бинарную маску) и переписываем в ту же переменную
        
        cv::inRange(frameHSVBall,
            cv::Scalar(H2_Ball.min, S_Ball_min, V_Ball_min),
            cv::Scalar(H2_Ball.max, 255, 255),
            frameHSVBall2); // ищем только вхождения нужного цвета (преобразуем в бинарную маску) и переписываем в ту же переменную
        frameHSVBall = frameHSVBall1 + frameHSVBall2;

        cv::cvtColor(frameTop, frameLABBall, cv::COLOR_BGR2Lab);// преобразуем полученный кадр в Lab
        std::vector<cv::Mat> vectorFrameLabBall; // создаем массив типа Mat (вектор - динамический массив). Нужны библиотеки vector и iostream
        cv::Mat maskABall, maskBBall; // создаем маски для осей А и В
        cv::split(frameLABBall, vectorFrameLabBall); // разделяем кадр в Lab на потоки
        cv::inRange(vectorFrameLabBall[1], A_Ball.min, A_Ball.max, maskABall); // проверяем вхождения в поток А
        cv::inRange(vectorFrameLabBall[2], B_Ball.min, B_Ball.max, maskBBall); // проверяем вхождения в поток И
        frameLABBall = maskABall | maskBBall; // объединяем результат в единую маску и записываем его

        frameLabHSVBall = frameHSVBall & frameLABBall; // объединяем маски HSV и Lab в единый кадр
        cv::morphologyEx(frameLabHSVBall, frameLabHSVBall, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 2); // чистим раз
        cv::morphologyEx(frameLabHSVBall, frameLabHSVBall, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 8); // чистим два
        cv::morphologyEx(frameLABBall, frameLABBall, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 2); // чистим раз
        cv::morphologyEx(frameLABBall, frameLABBall, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 8); // чистим два

        std::vector<std::vector<cv::Point>> contourBall; // создаем вектор векторов координат для контуров объекта. Вектор векторов - это двухмерный массив.
        cv::findContours( //Ищем контуры объекта
            frameLabHSVBall, // Откуда читаем
            contourBall, // куда выводим
            cv::RETR_EXTERNAL, // режим извлечения контуров. Берёт только внешние контуры. Варианты: RETR_LIST, RETR_TREE, RETR_CCOMP
            cv::CHAIN_APPROX_SIMPLE); // метод хранения точек контура. Оставляет только углы и ключевые точки. Вариант: CHAIN_APPROX_NONE (хранит каждый пиксель)

        double maxAreaBall = 0; // переменная для хранения максимального контура
        int maxIndexBall = -1; // номер максимального контура

        for (int n = 0; n < contourBall.size(); n++) // пока не перебрали все контуры
        {
            double areaBall = cv::contourArea(contourBall[n]); // пишем текущий контур
            if (areaBall > maxAreaBall) // если он больше максимального
            {
                maxAreaBall = areaBall; // обновляем максимальный
                maxIndexBall = n; // запоминаем индекс
            }
        }

        if (maxIndexBall != -1 && maxAreaBall > 200) // если объект обнаружен и больше 200, то ищем его центр
        {
            cv::Point2f objectCenterBall; // создаем переменную для записи координат
            float objectRadiusBall = 0; // создаем переменную для радиуса круга
            cv::minEnclosingCircle(contourBall[maxIndexBall], objectCenterBall, objectRadiusBall); // рассчитываем минимальную описанную окружность для найденного контура объекта
            cv::circle(frameTop, objectCenterBall, (int)objectRadiusBall, cv::Scalar(255, 0, 0), 2); // рисуем саму окружность на изначальном изображении
            cv::circle(frameTop, objectCenterBall, 3, cv::Scalar(0, 255, 0), -1); // рисуем центральную точку на изначальном изображении
        }

        if (mode == Mode::ARKANOID) // если режим Арканоид
        {
            cv::putText(frameTop, "Mode: 1 Arkanoid   2 RobotCal   3 BallCal",
                cv::Point(15, 28), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2); // рисуем текст с режимами
            cv::imshow("View", frameTop); // выводим
        }
        else if (mode == Mode::ROBOT_CAL)
        {
            cv::Mat tl = frameTop.clone();
            putText(tl, "Robot Calibration (2)", cv::Point(15, 28),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2); // выводим название режима
            putText(tl, "Keys: 1/3 switch modes", cv::Point(15, 55),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2); // выводим текст с инструкцией по переключению вкладок
            cv::Mat down_L_BGR, up_R_BGR, down_R_BGR; // создаем новые маски
            toBGR(frameHSV, down_L_BGR); // преобразуем маски в RGB
            toBGR(frameLAB, up_R_BGR);
            toBGR(frameLabHSV, down_R_BGR);

            putText(down_L_BGR, "HSV robot", cv::Point(15, 28), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2); // Вставляем названия окон
            putText(down_R_BGR, "Final (HSV|LAB) robot", cv::Point(15, 28), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
            putText(up_R_BGR, "LAB robot", cv::Point(15, 28), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);

            cv::Mat grid = makeGrid(tl, down_L_BGR, up_R_BGR, down_R_BGR, FIELD_W, FIELD_H); // создаем 4 окна
            imshow("View", grid); // отображаем кадр в окне с именем LabHSVVideo
        }
        else if (mode == Mode::BALL_CAL)
        {
            cv::Mat tl = frameTop.clone();
            putText(tl, "Ball Calibration (3)", cv::Point(15, 28),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2); // выводим название режима
            putText(tl, "Keys: 1/3 switch modes", cv::Point(15, 55),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2); // выводим текст с инструкцией по переключению вкладок
            cv::Mat down_L_BGR, up_R_BGR, down_R_BGR; // создаем новые маски
            toBGR(frameHSVBall, down_L_BGR); // преобразуем маски в RGB
            toBGR(frameLABBall, up_R_BGR);
            toBGR(frameLabHSVBall, down_R_BGR);

            putText(down_L_BGR, "HSV Ball", cv::Point(15, 28), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2); // Вставляем названия окон
            putText(down_R_BGR, "Final (HSV|LAB) Ball", cv::Point(15, 28), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
            putText(up_R_BGR, "LAB Ball", cv::Point(15, 28), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);

            cv::Mat grid = makeGrid(tl, down_L_BGR, up_R_BGR, down_R_BGR, FIELD_W, FIELD_H); // создаем 4 окна
            imshow("View", grid); // отображаем кадр в окне с именем LabHSVVideo
        }

        if (cv::waitKey(1) == 27) break; // ожидание 1 мс и проверка нажатия клавиши ESC
    }
    shutdownUDP(); // отключаем UDP
}