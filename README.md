**Robot Sumo Autonom | C++, Arduino, PlatformIO**

* **Obiectiv:** Proiectarea și implementarea software-ului pentru un robot de tip mini-sumo capabil să detecteze adversarii și să navigheze autonom într-un ring competițional.
* **Arhitectură Software:** Dezvoltat o logică de control bazată pe o mașină de stări (Search și Combat) pentru a gestiona comportamentul robotului în funcție de prezența inamicului.
* **Integrare Hardware:** Implementat rutine de procesare a datelor în timp real de la 5 senzori infraroșu (IR) pentru detecția 360° a adversarilor și 2 senzori de linie analogici pentru evitarea ieșirii din ring.
* **Abstractizare și Modularitate:** Creat o bibliotecă personalizată (`XMotionClass`) pentru abstractizarea componentelor hardware, gestionând controlul motoarelor prin semnale PWM, monitorizarea senzorilor și semnalizarea prin LED-uri de stare.
* **Tehnologii utilizate:** Programare C++ pe platformă Atmel AVR (Arduino Leonardo), utilizând mediul PlatformIO pentru gestionarea dependențelor și a procesului de build.
