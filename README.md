![logo of pg](https://cdn.pg.edu.pl/ekontakt-updated-theme/images/logo-pg-small.png)

![logo of ksa](https://eti.pg.edu.pl/documents/176593/23878975/eti-katedra-baner.jpg?t=1400069688565)

# Implementacja filtrów cyfrowych do estymacji orientacji robota mobilnego

| przedmiot | Roboty mobilne |  |
| --- | --- | --- |
| prowadzący | dr inż. Stanisław Raczyński |  |
| autorzy  | Robert Drozd <br> Marlena Gruba <br> Piotr Kluczek <br> Jakub Olesz | 160473 <br> 160225 <br> 160791 <br> 160475 |
| miejsce i data | Politechnika Gdańska, 3 grudnia 2018 roku |  |

## 1. 1.Wstęp

W projekcie realizowanym w ramach laboratorium robotów mobilnych celem jest implementacja filtrów cyfrowych do estymacji orientacji robota w przestrzeni na podstawie pomiarów z IMU ( _ang. Inertial Measurement Unit_). W ramach realizowanego zadania zostały zrealizowane:

- filtr Kalmana,
- rozszerzony filtr Klamana,
- filtr komplementarny,
- filtr Mahony&#39;ego.

Skorzystano z pomiarów znajdujących się w repozytorium GitHub ([https://github.com/SMAC-Group/imudata](https://github.com/SMAC-Group/imudata)), z zasugerowanego przez prowadzącego zbioru danych pomiarowych z płytki NavChip.

## 1. 2.Pomiar kąta przy wykorzystaniu danych z IMU

Dostępne były pomiary pochodzące z przyspieszeniomierza oraz z żyroskopu, co pozwoliło na estymację kątów przechylenia (z ang. Roll) Φ
 oraz pochylenia (z. ang. Pitch)
Θ
. Aby uzyskać zmierzone za pomocą pierwszego z czujników kąty, wykorzystano poniższe zależności:


<img src="https://latex.codecogs.com/svg.latex?\Large&space;\Theta=arcsin(−\frac{a_x}{g})," title="\Large \theta=arcsin(−\frac{a_x}{g})," />

<img src="https://latex.codecogs.com/svg.latex?\Large&space;\Phi=atan2(a_y,a_z)" title="\Large \Phi=atan2(a_y,a_z)" />

gdzie ax,ay,az
 reprezentują wartości rzutów wektora przyspieszenia ziemskiego
g
 na osie kolejno
x,y,z
. Do wyznaczenia kątów zmierzonych przez żyroskop, którego wyjściem jest wektor trzyelementowy zawierający prędkości kątowe w trzech osiach, posłużono się całkowaniem metodą prostokątów.

## 1. 3.Filtr Kalmana

Filtr Kalmana jest narzędziem pozwalającym na wykorzystanie sygnałów pomiarowych (np. położenia obiektu) o różnych wariancjach, pochodzących z niezależnych czujników, do uzyskania sygnału estymowanego, którego wariancja jest mniejsza od każdej z wariancji sygnałów składowych. Wpływ każdego z sygnałów pomierzonych, na sygnał wyjściowy z filtru Kalmana, jest tym mniejszy, im większa jest jego wariancja. Filtracja Kalmana odbywa się w kilku krokach:

- predykcja (przewidywanie kolejnej wartości wyjścia),
- obserwacja (pobranie danych z czujników),
- innowacja (różnica predykcji i sygnału pomiarowego),
- aktualizacja (obliczenie wyjścia estymowanego na podstawie predykcji, innowacji oraz dobranych wzmocnień).

Z wykorzystaniem pomiarów dokonana została implementacja filtracji Kalmana. Implementacji dokonano przy użyciu środowiska Matlab, wyniki znajdują się w pliku _kalman.m_.

## 1. 4.Filtr komplementarny

Filtr opiera się na analizie sygnałów z żyroskopu i akcelerometru połączonego z magnetometrem. W pracy do poprawnego określenia orientacji w przestrzeni wykorzystuje się pomiary z akcelerometrów oraz magnetometrów zakłócone sygnałem wysokoczęstotliwościowym, oraz z żyroskopów skażonych sygnałem wolnozmiennym. Dzieje się tak dlatego, że wskazania akcelerometru opierają się na pomiarze inercji elementu drgającego (przetworniki sejsmiczne) o transmitancji 2 rzędu. Z kolei pomiary żyroskopowe ukazują stały obrót wokół wszystkich osi mimo braku ruchu. W związku z tym przy całkowaniu tego pomiaru, wartość kąta Θ
 będzie się obsuwać stale.

W związku z tym filtr komplementarny zaprojektowano uwzględniając owe słabości przetworników poprzez wykorzystanie filtracji pomiarów z każdego z nich, aby pozbyć się szumów. Dokonuje się tego przez filtrację sygnałów filtrami dolnoprzepustowymi i górnoprzepustowymi komplementarnymi tak, aby spełniona była równość

HPF+LPF=1

Można stosować różne filtry, jednak zwykle wystarczy filtr pierwszego rzędu o transmitancji

<img src="https://latex.codecogs.com/svg.latex?\Large&space;LPF=\frac{1}{1+Ts}" title="\Large LPF=\frac{1}{1+Ts}" />

gdzie _T_ to stała czasowa określająca dynamikę filtru. Dalej zgodnie z zasadą komplementarności

<img src="https://latex.codecogs.com/svg.latex?\Large&space;HPF=\frac{Ts}{1+Ts}" title="\Large HPF=\frac{Ts}{1+Ts}" />

Aby zaimplementować powyższe filtry w dziedzinie dyskretnej należy przyjąć odpowiednie przybliżenie różniczki jako pierwszą różnicę wsteczną z czasem Δ
t równym częstotliwości próbkowania. Po odpowiednich przekształceniach postać równania filtru cyfrowego prezentuje się następująco

<img src="https://latex.codecogs.com/svg.latex?\Large&space;\hat{a}_k=p\hat{a}_{k-1}+(1-p)a_k^{AM}+p(a_k^G-a_{k-1}^G)," title="\Large \hat{a}_k=p\hat{a}_{k-1}+(1-p)a_k^{AM}+p(a_k^G-a_{k-1}^G)," />

<img src="https://latex.codecogs.com/svg.latex?\Large&space;p=\frac{T}{{\Delta}t+T}," title="\Large p=\frac{T}{{\Delta}t+T}" />

gdzie α^k
 oznacza wartość kąta estymowanego k-tej próbki a αkAM i αkG to odpowiednio obliczone na podstawie wzorów z rozdziału 2 wartości kątów wynikające z pomiarów akcelerometrem i magnetometrem oraz z żyroskopu. Stała czasowa _T_ to wartość określająca jak szybko ustala się odpowiedź filtra. Zbyt mała wartość będzie skutkowała większym wpływem żyroskopu, za duża, pominięciem go.

W związku z brakiem danych z magnetometru w podanym zestawie danych obliczone będą tylko wcześniej wymienione kąty. Wyniki powyższych rozważań zostały zaprezentowane w pliku _Complementary.m._ Zaimplementowany filtr zachowuje się zgodnie z oczekiwaniami.

## 1. 5.Filtr Mahony&#39;ego

W celu ustalenia orientacji obiektu, filtr Mahony&#39;ego implementuje pewną postać sterownika PI. Najpierw porównuje się ostatnią estymację orientacji z orientacją wyznaczoną w danej chwili czasu jedynie na podstawie odczytów z akcelerometru. Wyznacza się błąd poprzez obliczenie iloczynu wektorowego dwóch wektorów wspomnianych wektorów. Jeżeli wektory się pokrywają to błąd wynosi zero. Jeżeli nie są zorientowane identycznie, otrzymuje się niezerowy błąd, który zostaje podany do sterownika PI. Współczynnikach Kp i Ki są podawane jednorazowo przed rozpoczęciem filtracji. Wyjście ze sterownika dodaje się do aktualnego odczytu z żyroskopu. Tak zmodyfikowany wektor prędkości kątowej ∆ω
 zostaje przemnożony przez
∆t
 i dodany do ostatniej estymacji orientacji.

W pliku mahony\_navchip.m znajduje się implementacja filtru zastosowana do danych pomiarowych uzyskanych przy pomocy płytki NavChip. Dostępne były pomiary pochodzące z akcelerometru oraz z żyroskopu, co pozwoliło na estymację kątów przechylenia (z ang. Roll) Φ
 oraz pochylenia (z. ang. Pitch)
Θ
. Bez danych z magnetometru, kąt Yaw nie może być wyznaczony bezwzględnie i dlatego na wykresie obserwuje stał pozorny ruch wokół tej osi związany z niezerowym dryfem żyroskopu wynikającym z szumu pomiarowego i stałego uchybu sensora (a ang, bias). W pliku mahony\_2.m zastosowano filtr do danych z żyroskopu i akcelerometru udostępnionych przez Magwick&#39;a. W czasie odpowiadającym około 6500 próbek płytka IMU była przechylana od -90 do +90 stopni wokół każdej z osi.

