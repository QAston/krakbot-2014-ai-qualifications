# Zadanie kwalifikujące na krakrobot

## Kalendarz

* 0225✔ - startKrak
* 0225✔ - Wszystko dziala poprzez commit i zmiane kodu programu
* 0226✔ - Generator map
* 0226✔ - Prosty tester robota
* 0309 - Dead line

## Coding style

Trzymamy sie stylu zdefiniowanego w http://legacy.python.org/dev/peps/pep-0008
M.in. 4 spacje intendacji, nazwa_funkcji, NazwaKlasy

UWAGA - nie edytujemy nic z folderu krak, jest to submoduł git i zmiany nie bedą widoczne u innych!!!

## Todo
* napisać algorytmy robotów dla różnych poziomów dokładności sensorów:
    * perfectionist - 80% complete - radzi sobie w przypadku ultra-precyzyjnego ruchu (bez sensorów), oraz przy drobnych błedach w ruchu(sonar przy zakłuceniach distance_noise, gps w pozostałych)
        * trzeba napisać lepszy algorytm przeszukiwania mapy
        * lepsza heurystyka poruszania sie po mapie
        * dodać wykrywanie sytuacji w których robot sobie nie radzi aby można było zmienić algorytm
        * zoptymalizować kod
    * kretyn - ?? complete - radzi sobie w przypadku gdy nic nie działa jak trzeba
    * navigator - 0% complete - radzi sobie w przypadku średniego zakłucenia na wszystkich odczytach
        * ważny algorytm, pokrywa >50% możliwych kombinacji odczytów!!!!

* zintegrować algorytmy
    * trzeba napisać dynamiczne przełączanie sie pomiedzy algorytmami
        *robot zaczyna jako perfectionist, by gdy już nie jest czegoś pewnien bądz walnął w ściane przełączyć sie na navigator

* przetestować wszystko
* przejść kwalifikacje

## Pomysły na algorytm robota
* tu można wpisywać co ktoś wymyśli

## Problemy które muszą zostać rozwiązane w algorytmie robota
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
