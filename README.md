# Zadanie kwalifikujące na krakrobot

## Kalendarz

* 0225✔ - startKrak
* 0225 - Wszystko dziala poprzez commit i zmiane kodu programu
* 0226 - Generator map
* 0226 - Prosty tester robota
* 0227 - Obsluga ruchu robotem
* 0228 - Obsluga S1
* 0228 - Obsluga S2
* 0228 - Obsluga S3
* 0301 - Algorytm ruchu robota
* 0302 - Pierwszy gotowy program ruchu
* 0309 - Dead line

## Coding style

Trzymamy sie stylu zdefiniowanego w http://legacy.python.org/dev/peps/pep-0008
M.in. 4 spacje intendacji, nazwa_funkcji, NazwaKlasy

UWAGA - nie edytujemy nic z folderu krak, jest to submoduł git i zmiany nie bedą widoczne u innych!!!

## Todo

* potrzebny jest skrypt który bedzie wykonywał testy robota na podanych mapach i sprawdzał czy czasy po ostatniej zmianie sie nie pogorszyły
* potrzebny jest inny generator map - obecny generuje tylko labirynty
* trzeba napisać dobry program robota
    * potrzebne są jakieś metody probabilistyczne do radzenia sobie z niepewnością odczytów sensorów
* przejść kwalifikacje

## Pomysły na algorytm robota
* tu można wpisywać co ktoś wymyśli

## Problemy które muszą zostać rozwiązane w algorytmie robota
* zminimalizowanie liczby odczytów pola, aby zminimalizować kosz FIELD_TIME
    * sprawdzamy rodzaj pola przy wejsciu na nie, jesli jest szum to kilka razy (min potrzebna liczbe mozna pewnie wyliczyc)
* aby wiedziec kiedy zmienilismy pole trzeba przechowywac w jakis sposob pozycje na planszy
    * mozna wykorzystac gps, ale on szumi, co trzeba wziac pod uwage
    * mozna wykorzystac algebre liniowa do aktualizowania pozycji robota bez gps
    * mozna wykorzystac sensor odleglosci ale to tez zalezy od jego dokladnosci
    * mozna tez wykorzystac sensor typu pola, chociaz to niekoniecznie dobra metoda bo on moze rzadko zmieniac wartosc
* zmilimalizowanie liczby ruchow na planszy
    * trzeba przechowywać wewnetrzną reprezentacje planszy, aby np moc szybko wrócić do przeszukiwania po wpadnieciu w ślepą uliczke
    * wewnetrzna mapa musi byc szybko przeszukana, zeby robot duzo nie jezdzil
* zminimalizowanie liczby obrotów - obroty tez kosztują
    * trzeba wybierac optymalny kierunek obrotu
* wykorzystanie podpowiedzi z mapy
    * czasem jest podawana optymalna sciezka
    * czasem jest podawany kierunek wyjscia
    * czasem jest podawana odleglosc euklidesowa


##instalacja

* potrzebny python 2.7
* numpy http://sourceforge.net/projects/numpy
* pyqt http://www.lfd.uci.edu/~gohlke/pythonlibs/
* opcjonalnie scipy http://sourceforge.net/projects/scipy/


##linki i sznurki
kurs- https://www.udacity.com/wiki/cs373
