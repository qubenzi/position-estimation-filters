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

- filtr Kalmana (Marlena Gruba i Robert Drozd),
- filtr komplementarny (Piotr Kluczek),
- filtr Mahony&#39;ego (Jakub Olesz).

Skorzystano z pomiarów znajdujących się w repozytorium GitHub ([https://github.com/SMAC-Group/imudata](https://github.com/SMAC-Group/imudata)), z zasugerowanego przez prowadzącego zbioru danych pomiarowych z płytki NavChip.

## 1. 2.Pomiar kąta przy wykorzystaniu danych z IMU

Dostępne były pomiary pochodzące z przyspieszeniomierza oraz z żyroskopu, co pozwoliło na estymację kątów przechylenia (z ang. Roll) Φ
 oraz pochylenia (z. ang. Pitch)
Θ
. Aby uzyskać zmierzone za pomocą pierwszego z czujników kąty, wykorzystano poniższe zależności:


<img src="https://latex.codecogs.com/svg.latex?\Large&space;\Theta=arcsin(−\frac{-a_x}{g})," title="\Large \theta=arcsin(−\frac{a_x}{g})," />

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

Z wykorzystaniem pomiarów dokonana została implementacja filtracji Kalmana. Implementacji dokonano przy użyciu środowiska Matlab, wyniki znajdują się w pliku _kalman.m_. Na poniższych wykresach przedstawiono wyniki filtracji. W czasie 0.3 s wyjście filtru ustaliło się. Szumy zakłóceniowe z akcelerometru oraz dryf żyroskopu zostały skutecznie odfiltrowane.

Kąty przechylenia oraz pochylenia uzyskane za pomocą akceleromteru, żyroskopu oraz po filtracji filtrem Kalmana:
<img src="https://raw.githubusercontent.com/legwan/position-estimation-filters/master/Pics/kalman1.png" />
Początkowe 0,6 s pracy algorytmu:
<img src="https://raw.githubusercontent.com/legwan/position-estimation-filters/master/Pics/kalman2.png" />

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

W związku z brakiem danych z magnetometru w podanym zestawie danych obliczone będą tylko wcześniej wymienione kąty. Wyniki powyższych rozważań zostały zaprezentowane w pliku _Complementary.m._ Zaimplementowany filtr zachowuje się zgodnie z oczekiwaniami:

Dla T = [0.01 0.01] :
![Alt text](Pics/Complementary_all.bmp?raw=true )
![Alt text](Pics/Complementary_theta_effect.bmp?raw=true )
![Alt text](Pics/Complementary_theta_trasition.bmp?raw=true )

Jak widać procesy przejściowe są bardzo krótkie i wartość początkowa filtru jest dość bliska wartości pochodzących z akcelerometru. Poniżej przedstawiono wynik dla T = [0.1 0.1] :

![Alt text](Pics/Complementary_theta_effect2.bmp?raw=true )
![Alt text](Pics/Complementary_theta_trasition2.bmp?raw=true )

## 1. 5.Filtr Mahony&#39;ego

<b>Działanie filtru</b>

W celu ustalenia orientacji obiektu, filtr Mahony&#39;ego implementuje pewną postać sterownika PI. Najpierw porównuje się ostatnią estymację orientacji z orientacją wyznaczoną w danej chwili czasu jedynie na podstawie odczytów z akcelerometru. Wyznacza się błąd poprzez obliczenie iloczynu wektorowego dwóch wektorów wspomnianych wektorów. Jeżeli wektory się pokrywają to błąd wynosi zero. Jeżeli nie są zorientowane identycznie, otrzymuje się niezerowy błąd, który zostaje podany do sterownika PI. Współczynnikach Kp i Ki są podawane jednorazowo przed rozpoczęciem filtracji. Wyjście ze sterownika dodaje się do aktualnego odczytu z żyroskopu. Tak zmodyfikowany wektor prędkości kątowej ∆ω
 zostaje przemnożony przez
∆t
 i dodany do ostatniej estymacji orientacji. 

<b>Omówienie wyników</b>

W pliku mahony\_navchip.m znajduje się implementacja filtru zastosowana do danych pomiarowych uzyskanych przy pomocy płytki NavChip. Dostępne były pomiary pochodzące z akcelerometru oraz z żyroskopu, co pozwoliło na estymację kątów przechylenia (z ang. Roll) Φ
 oraz pochylenia (z. ang. Pitch)
Θ
. Bez danych z magnetometru, kąt Yaw nie może być wyznaczony bezwzględnie i dlatego na wykresie obserwuje stał pozorny ruch wokół tej osi związany z niezerowym dryfem żyroskopu wynikającym z szumu pomiarowego i stałego uchybu sensora (a ang, bias). 

Poniższe wykresy ilustrują działanie części całkującej filtru. Pierwsze dwa z nich przedstawiają działanie filtru dla <i>kp</i>=3 oraz <i>ki</i>=0. Wstępna korekcja średniego uchybu żyroskopu jest wyłączona. Wyjście filtru po pewnym czasie się stabilizuje, ale z niezerowym uchybem. 
<img src="https://raw.githubusercontent.com/legwan/position-estimation-filters/master/Pics/mahony_3_0_uchyb.png" />
<p align="center"><i>Filtr Mahony&#39;ego - kp = 3, ki = 0</i></p>

Druga para wykresów przedstawia działanie filtru dla kp=3 oraz ki=5. Wyjście filtru się stabilizuje, a element całkujący powoduje, że uchyb ustalony dąży do zera.
<img src="https://raw.githubusercontent.com/legwan/position-estimation-filters/master/Pics/mahony_3_5_korekcja_uchybu.png" />
<p align="center"><i>Filtr Mahony&#39;ego - kp = 3, ki = 5</i></p>

Kolejne wykresy przedstawia działanie filtru dla <i>ki</i>=4 oraz <i>kp</i> wynoszącego kolejno: 10, 20 i 60. Zastosowanie większego wzmocnienia powoduje, że filtr szybciej osiąga stan ustalony. Większe wzmocnienie oznacza również, że filtr będzie bardziej podatny na szum pomiarowy. Dla kp=60 szum na wyjściu filtru jest bardzo podobny do szumu związanego z danymi akcelerometru.
<img src="https://raw.githubusercontent.com/legwan/position-estimation-filters/master/Pics/mahony_bias_off_10_4_1.png" />
<img src="https://raw.githubusercontent.com/legwan/position-estimation-filters/master/Pics/mahony_bias_off_10_4_2.png" />
<img src="https://raw.githubusercontent.com/legwan/position-estimation-filters/master/Pics/mahony_bias_off_10_4_3.png" />
<p align="center"><i>Filtr Mahony&#39;ego - kp = 10, ki = 4</i></p>

<img src="https://raw.githubusercontent.com/legwan/position-estimation-filters/master/Pics/mahony_bias_off_20_4_1.png" />
<img src="https://raw.githubusercontent.com/legwan/position-estimation-filters/master/Pics/mahony_bias_off_20_4_2.png" />
<img src="https://raw.githubusercontent.com/legwan/position-estimation-filters/master/Pics/mahony_bias_off_20_4_3.png" />
<p align="center"><i>Filtr Mahony&#39;ego - kp = 20, ki = 4</i></p>

<img src="https://raw.githubusercontent.com/legwan/position-estimation-filters/master/Pics/mahony_bias_off_60_4_1.png" />
<img src="https://raw.githubusercontent.com/legwan/position-estimation-filters/master/Pics/mahony_bias_off_60_4_2.png" />
<img src="https://raw.githubusercontent.com/legwan/position-estimation-filters/master/Pics/mahony_bias_off_60_4_3.png" />
<p align="center"><i>Filtr Mahony&#39;ego - kp = 60, ki = 4</i></p>

W pliku mahony\_2.m zastosowano filtr do danych z żyroskopu i akcelerometru udostępnionych przez Magwick&#39;a. W czasie odpowiadającym około 7000 próbek płytka IMU była przechylana od -90 do +90 stopni wokół każdej z osi. Dwa poniższe wykresy ilustrują dane z sensorów oraz działanie filtru dla kp = 1, ki = 1.

<img src="https://raw.githubusercontent.com/legwan/position-estimation-filters/master/Pics/mahony2_sensors.png" />
<p align="center"><i>Obracanie IMU - dane z sensorów</i></p>

<img src="https://raw.githubusercontent.com/legwan/position-estimation-filters/master/Pics/mahony2_filter.png" />
<p align="center"><i>Obracanie IMU - filtr Mahony&#39;ego</i></p>

<b> instrukcja uruchamiania i edycji plików </b>
M-pliki należy otwierać w środowisku obliczeniowym Matlab. Wymagane jest także rozpakowanie katalogu navchip.zip. W plikach mahony\_2 oraz mahony\_navchip N można zmienić ilość próbek które mają być poddane filtracji <i>(zmienna "samples")</i>, wartości współczynników filtru (zmienne "kp" oraz "ki"), a w pliku mahony\_navchip.m można dodatkowo ustawić flagę bias_correction (0 - korekcja wyłączona, 1 - korekcja włączona). Włączenie tej opcji usuwa przed zastosowaniem filtracji skłądową stałą żyroskopu. Dla każdej z osi jest obliczana średnia arytmetyczna wszystkich próbek, a następnie odejmowana od zarejestrowanych danych.

<i>W projekcie zastosowano bibliotekę do obliczeń na quaternionach przygotowaną przez Magwick&#39;a.</i>
