#ifndef INC_STM32G_UART_HPP_
#define INC_STM32G_UART_HPP_

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

/*
 * 	ВНИМАНИЕ!
 *		- Для совместимости требуется проект C++ (main.cpp)!
 *		- Библиотека не настраивает системное тактирование и GPIO!
 *
 *	Библиотека для работы с UART / USART / LPUART на STM32G4xx и STM32G0xx:
 *		- Доступен вывод всех (основных) типов данных, printf()
 *		- Режим работы ТОЛЬКО асинхронный (UART), в формате 8N1
 *		- Реализованы кольцевые буферы, обслуживание по прерыванию
 *		- Для тактирования USART-ов используется HSI16 (по умолчанию) или SYSCLK (16-170 MHz)
 *		- Неблокирующая отправка данных, пока есть место в TX-буфере
 *		- Блокирущая отправка данных, если TX-буфер заполнился
 *
 *	API:
 *	1. Создание объекта uart (на примере LPUART1):
 *		STM32G4_UART <UART_FULL> uart(LPUART1);				// Полнодуплексный, буферы: [256 байт RX / 256 байт TX]
 *		STM32G4_UART <UART_FULL, 128, 512> uart(LPUART1);	// Полнодуплексный, буферы: [128 байт RX / 512 байт TX]
 *		STM32G4_UART <UART_RX, 512, 1> uart(LPUART1);		// Только прием, буферы: [512 байт RX / 1 байт TX]
 *		STM32G4_UART <UART_TX, 1, 512> uart(LPUART1);		// Только передача, буферы: [1 байт RX / 512 байт TX]
 *
 *	2. Создание обработчика прерывания (на примере LPUART1):
 *		extern "C" void LPUART1_IRQHandler () {uart.IRQ();}	// Создаем обработчик и передаем ему метод IRQ();
 *
 *	3. Инициализация и управление:
 *		uart.init(baud);				// Инициализировать, тактируя от HSI16. baud - скорость [4800...2000000] бод/с
 *		uart.init(baud, SYSCLK);		// Инициализировать, тактируя от SYSCLK. SYSCLK - частота ядра 16...170МГц
 *		uart.deInit();					// Полный сброс и отключение UART, включая тактирование
 *		uart.setBaudrate(baud); 		// Изменение baudrate после инициализации. Параметры аналогичны uart.init();
 *		uart.setBaudrate(baud, SYSCLK); // Изменение baudrate после инициализации. Вариант с тактированием от SYSCLK
 *		uart.rxEnable(state);			// Аппаратный вкл. (true) / выкл. (false) приемника uart
 *		uart.txEnable(state);			// Аппаратный вкл. (true) / выкл. (false) передатчика uart
 *		uart.enable(state);				// Аппаратный вкл. (true) / выкл. (false) модуля uart
 *		uart.swap(state);				// Поменять контакты RX/TX местами (true), оставить по умолчанию (false)
 *		uart.reset();					// Аппаратный сброс UART и буферов RX/TX
 *		uart.IRQ();						// Сервис прерываний UART, поместить в созданный обработчик указанного UART
 *
 *	4. Отправка данных (печать):
 *		uart.availableToWrite();		// Вернет true, если есть место для записи в буфер TX
 *		uart.printf(fmt, ...);			// Форматированная печать (аналог. prinf(...))
 *		uart.write(data);				// Отправить байт, вернет false если передатчик выключен, true в случае успеха
 *		uart.write(buf, len);			// Отправить массив байт, возращаемое значение аналогично uart.write();
 *		uart.println();					// Перенос строки (символы \r и \n)
 *		uart.print('c');				// Печать символа char
 *		uart.print("String");			// Печать c-string
 *		uart.print(value);				// Печать числа int8_t, uint8_t, int16_t, uint16_t, int32_t, uint32_t в десятичном виде
 *		uart.print(value, 2);			// Печать целого числа в двоичном виде (8 - OCT, 16 - HEX и т.д.)
 *		uart.print(value);				// Печать float, double с двумя знаками после запятой
 *		uart.print(value, 3);			// Печать float, double с тремя знаками (3, 4, 5 и т.д.)
 *		uart.printBIN(value);			// Печать uint8_t, uint16_t, uint32_t в двоичном виде с префиксом и разделением байт
 *		uart.txFlush();					// Очистка буфера TX
 *
 *	5. Чтение данных:
 *		uart.available();				// Вернет кол-во байт в буфере RX (доступных для чтения)
 *		uart.read();					// Прочитать 1 байт из буфера RX (и удалить его из буфера RX)
 *		uart.read(buf, len);			// Прочитать нужное кол-во байт в массив байт
 *		uart.peek();					// Прочитать 1 байт, но не удалить из буфера RX
 *		uart.rxFlush();					// Очистка буфера RX
*/

#ifndef PRINTF_BUF_SIZE
#define PRINTF_BUF_SIZE (128)			// Размер локального буфера для uart.printf(...);
#endif

#define UART_RX 		(USART_CR1_RE)
#define UART_TX 		(USART_CR1_TE)
#define UART_FULL		(UART_RX | UART_TX)

#ifndef WR_BIT
#define WR_BIT(REG, BIT, X) (X ? SET_BIT(REG, BIT) : CLEAR_BIT(REG, BIT))
#endif

template <uint8_t MODE, int RXSZ = 256, int TXSZ = 256> class STM32G4_UART {
public:

	/* ================================ Системное: инициализация и управление ================================ */
//, MODIFY_REG(RCC -> CCIPR, RCC_CCIPR_LPUART1SEL, RCC_CCIPR_LPUART1SEL_1)
	STM32G4_UART (USART_TypeDef *UART) : UARTx(UART){}

	void init(uint32_t baudrate, uint32_t clk = 0UL){
		/* 1. Полный сброс выбранного UART через RCC*/
		reset();

		/* 2. Если работаем от HSI16, он должен быть включен (на всякий случай) */
		if(!clk && !(RCC -> CR & RCC_CR_HSION)){
			RCC -> CR |= RCC_CR_HSION;							// Включаем HSI16
			while(!(RCC -> CR & RCC_CR_HSIRDY));				// Ждем выхода на режим
		}

		/* 3. Выбор тактирования UART: если указан clk - от SYSCLK, если не указан - от HSI16 */
		switch((uint32_t) UARTx){
			case LPUART1_BASE:  MODIFY_REG(RCC -> CCIPR, RCC_CCIPR_LPUART1SEL, clk != 0 ? RCC_CCIPR_LPUART1SEL_0 : RCC_CCIPR_LPUART1SEL_1); break;
			case USART1_BASE:	MODIFY_REG(RCC -> CCIPR, RCC_CCIPR_LPUART1SEL, clk != 0 ? RCC_CCIPR_USART1SEL_0 : RCC_CCIPR_USART1SEL_1); 	break;
			case USART2_BASE:	MODIFY_REG(RCC -> CCIPR, RCC_CCIPR_LPUART1SEL, clk != 0 ? RCC_CCIPR_USART2SEL_0 : RCC_CCIPR_USART2SEL_1); 	break;
			case USART3_BASE:	MODIFY_REG(RCC -> CCIPR, RCC_CCIPR_LPUART1SEL, clk != 0 ? RCC_CCIPR_USART3SEL_0 : RCC_CCIPR_USART3SEL_1); 	break;
			case UART4_BASE:	MODIFY_REG(RCC -> CCIPR, RCC_CCIPR_LPUART1SEL, clk != 0 ? RCC_CCIPR_UART4SEL_0 : RCC_CCIPR_UART4SEL_1); 	break;
			case UART5_BASE:	MODIFY_REG(RCC -> CCIPR, RCC_CCIPR_LPUART1SEL, clk != 0 ? RCC_CCIPR_UART5SEL_0 : RCC_CCIPR_UART5SEL_1); 	break;
			default: return;
		}

		/* 4. Включение тактирования UART*/
		switch((uint32_t) UARTx){
			case LPUART1_BASE:  RCC-> APB1ENR2 |= RCC_APB1ENR2_LPUART1EN; 	break;
			case USART1_BASE:	RCC-> APB2ENR  |= RCC_APB2ENR_USART1EN; 	break;
			case USART2_BASE:	RCC-> APB1ENR1 |= RCC_APB1ENR1_USART2EN; 	break;
			case USART3_BASE:	RCC-> APB1ENR1 |= RCC_APB1ENR1_USART3EN; 	break;
			case UART4_BASE:	RCC-> APB1ENR1 |= RCC_APB1ENR1_UART4EN; 	break;
			case UART5_BASE:	RCC-> APB1ENR1 |= RCC_APB1ENR1_UART5EN; 	break;
			default: return;
		}

		/* 5. Включение приемника / передатчика и прерываний */
		UARTx -> CR1 = MODE;								  	// Включение приемника и передатчика (если указано)
		if(MODE & UART_RX) UARTx -> CR1 |= USART_CR1_RXNEIE;	// Прерывание по приему (если нужно)
		if(MODE & UART_TX) UARTx -> CR1 |= USART_CR1_TCIE;		// Прерывание по передаче (если нужно)

		/* 6. Остальные параметры по умолчанию, OVERRUN выкл. */
		UARTx -> CR2 = 0;										// По умолчанию
		UARTx -> CR3 = USART_CR3_OVRDIS;						// Отключаем проверку OVERRUN

		/* 7. Установка baudrate */
		setBaudrate(baudrate, clk);

		/* 8. Включение блока UART */
		enable(true);

		/* 9. Включение глобального прерывания UART */
		switch((uint32_t) UARTx){
			case LPUART1_BASE: 	NVIC_EnableIRQ(LPUART1_IRQn); 	break;
			case USART1_BASE:	NVIC_EnableIRQ(USART1_IRQn);	break;
			case USART2_BASE:	NVIC_EnableIRQ(USART2_IRQn); 	break;
			case USART3_BASE:	NVIC_EnableIRQ(USART3_IRQn);	break;
			case UART4_BASE:	NVIC_EnableIRQ(UART4_IRQn);		break;
			case UART5_BASE:	NVIC_EnableIRQ(UART5_IRQn);  	break;
		}
	}

	void deInit(void){
		/* 1. Выключение глобального прерывания UART */
		switch((uint32_t) UARTx){
			case LPUART1_BASE: 	NVIC_DisableIRQ(LPUART1_IRQn); 	break;
			case USART1_BASE:	NVIC_DisableIRQ(USART1_IRQn);	break;
			case USART2_BASE:	NVIC_DisableIRQ(USART2_IRQn); 	break;
			case USART3_BASE:	NVIC_DisableIRQ(USART3_IRQn);	break;
			case UART4_BASE:	NVIC_DisableIRQ(UART4_IRQn);	break;
			case UART5_BASE:	NVIC_DisableIRQ(UART5_IRQn);   	break;
		}

		/* 2. Полный сброс выбранного UART через RCC*/
		reset();

		/* 3. Выключение тактирования UART */
		switch((uint32_t) UARTx){
			case LPUART1_BASE: 	RCC-> APB1ENR2 &= ~RCC_APB1ENR2_LPUART1EN; 	break;
			case USART1_BASE:	RCC-> APB2ENR  &= ~RCC_APB2ENR_USART1EN; 	break;
			case USART2_BASE:	RCC-> APB1ENR1 &= ~RCC_APB1ENR1_USART2EN;	break;
			case USART3_BASE:	RCC-> APB1ENR1 &= ~RCC_APB1ENR1_USART3EN; 	break;
			case UART4_BASE:	RCC-> APB1ENR1 &= ~RCC_APB1ENR1_UART4EN;	break;
			case UART5_BASE:	RCC-> APB1ENR1 &= ~RCC_APB1ENR1_UART5EN;   	break;
		}
	}

	void setBaudrate(uint32_t baudrate, uint32_t clk = 0UL){
		if(!clk) clk = 16E6;											// Если не указан SYSCLK, используем HSI16
		if(baudrate < 4800) baudrate = 4800;							// Ограничение скорости снизу
		else if (baudrate > 2E6) baudrate = 2E6;						// Ограничение скорости сверху

		bool state = (UARTx -> CR1 & USART_CR1_UE);						// Сохраняем старое состояние
		enable(false);													// Выключаем UART

		if(UARTx != LPUART1){											// Все кроме LPUART1 (USART1...UART5)
			UARTx -> PRESC = 0;											// Делитель не нужен
			UARTx -> BRR = ((uint32_t) clk / baudrate);					// Просто делим и получаем BRR

		} else if (clk) {												// Для LPUART1, clk указан (SYSCLK)
			UARTx -> PRESC = 0b0100;									// Оптимальный делитель для 16...170МГц: /8
			UARTx -> BRR = ((uint32_t)(clk / 8) / baudrate) * 256UL;	// BRR = ((CLK / 8) / baudrate) * 256

		} else {														// Для LPUART1, clk не указан (HSI16)
			UARTx -> PRESC = 0b0000;									// Делитель не нужен
			UARTx -> BRR = ((uint32_t) clk  / baudrate) * 256UL;		// BRR = (CLK / baudrate) * 256
		}

		enable(state);													// Включаем обратно, если нужно
	}


	void rxEnable(bool state){ WR_BIT(UARTx -> CR1, USART_CR1_UE, USART_CR1_RE);} 	// Вкл. / выкл. приемника
	void txEnable(bool state){ WR_BIT(UARTx -> CR1, USART_CR1_UE, USART_CR1_TE);} 	// Вкл. / выкл. передатчика
	void enable(bool state){ WR_BIT(UARTx -> CR1, USART_CR1_UE, state);}			// Вкл. / выкл. модуля UART
	void swap(bool state){WR_BIT(UARTx -> CR2, USART_CR2_SWAP, state);}				// Вкл. / выкл. свопа RX/TX

	void reset(void){
		/* Сброс положения кольцевых буферов */
		txFlush();
		rxFlush();

		/* Принудительная очистка области буферов */
		memset(rxFifo, 0, RXSZ);
		memset(txFifo, 0, TXSZ);

		/* Аппаратный сброс регистров нужного UART через RCC: устаналиваем и сбрасываем бит RESET UARTx*/
		switch((uint32_t) UARTx){
			case LPUART1_BASE: 	RCC -> APB1RSTR2 |= RCC_APB1RSTR2_LPUART1RST,  	RCC -> APB1RSTR2 &= ~RCC_APB1RSTR2_LPUART1RST;	break;
			case USART1_BASE:	RCC -> APB2RSTR  |= RCC_APB2RSTR_USART1RST, 	RCC -> APB2RSTR  &= ~RCC_APB2RSTR_USART1RST;  	break;
			case USART2_BASE:	RCC -> APB1RSTR1 |= RCC_APB1RSTR1_USART2RST, 	RCC -> APB1RSTR1 &= ~RCC_APB1RSTR1_USART2RST;	break;
			case USART3_BASE:	RCC -> APB1RSTR1 |= RCC_APB1RSTR1_USART3RST, 	RCC -> APB1RSTR1 &= ~RCC_APB1RSTR1_USART3RST;	break;
			case UART4_BASE:	RCC -> APB1RSTR1 |= RCC_APB1RSTR1_UART4RST,		RCC -> APB1RSTR1 &= ~RCC_APB1RSTR1_UART4RST;	break;
			case UART5_BASE:	RCC -> APB1RSTR1 |= RCC_APB1RSTR1_UART5RST, 	RCC -> APB1RSTR1 &= ~RCC_APB1RSTR1_UART5RST;  	break;
			default: return;
		}
	}

	/* Поллинг: в прерывании и вручную */	
	void IRQ(void){
		/*  Transmission complete: передача завершена */
		if(UARTx -> ISR & USART_ISR_TC){
			UARTx -> ICR = USART_ICR_TCCF;				// Сбросить флаг
			if(txFifoTail != txFifoHead){				// Если буфер не пуст
				UARTx -> TDR = txFifo[txFifoTail];  	// Передаем байт
				txFifoTail = (txFifoTail + 1) % TXSZ; 	// Двигаем хвост
			}
		}

		/* RX not empty: в приемном буфере что-то есть */
		if(UARTx -> ISR & USART_ISR_RXNE){
			UARTx -> RQR = USART_RQR_RXFRQ;				// Сбросить флаг
			uint8_t c = UARTx -> RDR;					// В любом случае читаем
			int i = (rxFifoHead + 1) % RXSZ;			// Положение нового значения в буфере
			if(i != rxFifoTail){						// Только если есть свободное место
				rxFifo[rxFifoHead] = c;  				// Пишем в буфер
				rxFifoHead = i;             			// Двигаем голову
			}
		}
	}
	
	/* ======================================= Отправка данных и печать  ===================================== */

	/* Вернет true, если есть куда писать данные на отправку */
	bool availableToWrite(void){
		if((txFifoHead + 1) % TXSZ != txFifoTail) return 1;
		return 0;
	}

	/* Форматированный вывод строки в порт, полный аналог printf(...) */
	int printf(char *fmt, ...){
		int result;
		char buf[PRINTF_BUF_SIZE];					//	Буфер под строку
		va_list param;								//	Список параметров
		va_start(param, fmt);						//	Начинаем с единственного параметра fmt
		result = vsprintf(buf, fmt, param);			//	Получаем строку
		va_end(param);								//	Конец списка параметров
		print(buf);									// 	Печать строки
		return result;								//	Возвращаем результат
	}

	/* Отправить 1 байт в порт (записать в буфер отправки) */
	bool write(uint8_t c){
		if(!(UARTx -> CR1 & UART_TX)) return 0;		// Если передатчик отключен - выходим
		int i = (txFifoHead + 1) % TXSZ;			// Положение нового значения в буфере
		while(i == txFifoTail);						// Ждем освобождения места в буфере
		txFifo[txFifoHead] = c;  					// Пишем в буфер
		txFifoHead = i;             				// Двигаем голову

		/* Надежный костыль: принудительно отправляем первый байт после простоя */
		if(UARTx -> ISR & USART_ISR_TXE){			// Регистр отправки пуст
			UARTx -> ICR = USART_ICR_TCCF;			// Сбрасываем флаг (на всякий случай)
			UARTx -> TDR = txFifo[txFifoTail];  	// Передаем первый байт вручную
			txFifoTail = (txFifoTail + 1) % TXSZ; 	// Двигаем хвост
		}

		return 1;
	}

	/* Отправить массив байт длинной len в порт */
	bool write(uint8_t *buf, uint16_t len){
		for(uint16_t i = 0; i < len; i++){
			if(!write(buf[i])) return 0;
		} return 1;
	}

	/* Перенос строки */
	void println(void){
		write('\r');
		write('\n');
	}

	/* Напечатать символ char */
	void print(char c){
		write(c);
	}

	/* Напечатать c-string  */
	void print(char *cstr){
		for(uint16_t i = 0; cstr[i]; i++){
			write(cstr[i]);
		}
	}

	/* Печать int8_t и int16_t со знаком и без */
	void print(uint8_t data, uint8_t base = 10)  {print((uint32_t) data, base);}
	void print(int8_t data, uint8_t base = 10)   {print((int32_t) data, base);}
	void print(uint16_t data, uint8_t base = 10) {print((uint32_t) data, base);}
	void print(int16_t data, uint8_t base = 10)  {print((int32_t) data, base);}

	/* Напечатать целое число со знаком */
	void print(int32_t data, uint8_t base = 10){
		if(data < 0){								// Отрицательное число
			write('-'); 							// Отправляем знак минус
			data = -data;							// Избавляемся от знака
		} print((uint32_t) data, base);			 	// Печатать число как положительное
	}

	/* Напечатать целое число без знака */
	void print(uint32_t data, uint8_t base = 10){
		char buf[8 * sizeof(long) + 1];				// Создаем буфер
		char *str = &buf[sizeof(buf) - 1];			// Создаем строку в виде указателя

		*str = '\0';								// Добавляем конец строки
		if (base < 2) base = 10;					// Защита от дурака
		do {										// Перевод числа в строку
			char c = data % base;
			data /= base;
			*--str = c < 10 ? c + '0' : c + 'A' - 10;
		} while (data);

		print((char*)str);							// Печать в порт
	}

	/* Напечатать число float */
	void print(float data, uint8_t dec = 2){print((double) data, dec);}

	/* Напечатать число double */
	void print(double data, uint8_t dec = 2){
		if(data < 0){								// Отрицательное число
			write('-'); 							// Отправляем знак минус
			data = -data;							// Избавляемся от знака
		}

	    uint32_t integer = data;					// Берем целую часть
	    print(integer, 10);							// Отправляем целую часть
	    write('.');									// Отправляем точку
	    data -= integer;							// Отнимаем целую часть
	    for (uint8_t i = 0; i < dec; i++) {			// Проходимся нужное кол-во раз
	        data *= 10.0;							// Вынимаем 1 знак
	        print((uint32_t)data);					// Отправляем 1 знак
	        data -= (uint32_t)data;					// Отнимаем отправленный знак
	    }
	}

	/* Печать unsigned char в бинарном виде */
	void printBIN(uint8_t data){
		print((char*)"0b");							// Выводим префикс
		for(uint8_t i = 0; i < 8; i++){				// 8 бит
			write(data & 0x80 ? '1' : '0');			// От старшего к младшему
			data = data << 1;						// Сдвигаем байт слево
		}
	}

	/* Печать unsigned int в бинарном виде */
	void printBIN(uint16_t data){
		print((char*)"0b");							// Выводим префикс
		for(uint8_t i = 0; i < 16; i++){			// 8 бит
			write(data & 0x8000 ? '1' : '0');		// От старшего к младшему
			data = data << 1;						// Сдвигаем байт слево
			if(i == 7) write('\'');					// Разделяем на 2 байта
		}
	}

	/* Печать unsigned long в бинарном виде */
	void printBIN(uint32_t data){
		print((char*)"0b");							// Выводим префикс
		for(uint8_t i = 0; i < 32; i++){			// 8 бит
			write(data & 0x80000000 ? '1' : '0');	// От старшего к младшему
			data = data << 1;						// Сдвигаем байт слево
			if(i == 7 || i == 15 || i == 23){		// Разделяем на 4 байта
				write('\'');
			}
		}
	}


	/* Очистка выходного буфера */
	void txFlush(void){
		txFifoTail = txFifoHead = 0;
	}
	
	/* ======================================== Чтение данных из порта  ====================================== */

	/* Вернет количество байт в буфере на чтение */
	uint16_t available(void){
		return (RXSZ + rxFifoHead - rxFifoTail) % RXSZ;
	}

	/* Прочитать данные из буфера в массив */
	void read(uint8_t *buf, uint16_t len){
		for(uint16_t i = 0; i < len; i++){			// Сколько нужно байт
			buf[i] = read();						// Читаем и пишем в массив
		}
	}

	/* Прочитать байт из буфера */
	uint8_t read(void){
		if (rxFifoTail == rxFifoHead) return 0;   	// Буфер пуст
		uint8_t data = rxFifo[rxFifoTail];      	// Берём с хвоста
		rxFifoTail = (rxFifoTail + 1) % RXSZ; 		// Двигаем хвост
		return data;
	}

	/* Прочитать, но не удалить байт из буфера */
	uint8_t peek(void){
		return rxFifo[rxFifoTail];					// Читаем крайний байт
	}

	/* Очистить приемный буфер */
	void rxFlush(void){
		rxFifoTail = rxFifoHead = 0;
	}
	
private:
	uint8_t rxFifo[RXSZ];							// Буфер на прием
	uint8_t txFifo[TXSZ];							// Буфер на передачу
	int rxFifoHead = 0, rxFifoTail = 0;				// Голова и хвост
	int txFifoHead = 0, txFifoTail = 0;				// Голова и хвост
	USART_TypeDef *UARTx;							// Указатель на блок uart
};
 
#endif
