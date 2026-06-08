//Машина едет вперед (или назад, если перепутаны пины мотора хода), пока включено питание.
//Плата ARDUINO NANO
//Пины мотора руля (НЕ ИСПОЛЬЗУЮТСЯ)
const int ENA = 3;  //ШИМ скорости руля
const int IN1 = 4;  //направление 1
const int IN2 = 6;  //направление 2

//Пины мотора хода
const int ENB = 5;  //ШИМ скорости хода
const int IN3 = 7;  //направление 1
const int IN4 = 8;  //направление 2

void setup() {
	//Настраиваем все пины как выходы
	pinMode(ENA, OUTPUT);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(ENB, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);

	//Мотор руля
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
	analogWrite(ENA, 0);// скорость 0

	// --- Ходовой мотор: полный вперёд ---
	digitalWrite(IN3, HIGH);  // направление 1
	digitalWrite(IN4, LOW);   // направление 2 инверсно
	analogWrite(ENB, 255);    // скорость максимальная
}

void loop() {}