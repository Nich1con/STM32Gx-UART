# STM32Gx-UART
Библиотека для максимально удобного использования аппаратных UART на **STM32G4xx** и **STM32G0xx**

### Особенности:
- Работает с аппаратными **LPUART1**, **USART1...USART3**, **UART4...UART5**
- Доступен только асинхронный (UART) режим в формате **8N1**, скорость **4800...2000000** Бод/с
- Реализован вывод основных типов данных методом **print()**
- Реализован **printf()** - печать форматированной строки
- Реализован вывод чисел 8-32 бита в **полном** двоичном виде 
- Независимые FIFO буферы, конфигурируемый размер 
- Полностью фоновая работа по прерываниям
- Неблокирующие функции печати, пока есть место в буфере
- Тактирование от **HSI16** (по умолчанию) и **SYSCLK** (при желании)
- Максимально удобная работа со всеми UART

### ВНИМАНИЕ!
- Совместимо **только** с проектами **C++**! 
- Библиотека **не настраивает** GPIO и системное тактирование!

### Выбор тактирования:
Любой из UART может тактироваться от **HSI16** (по умолчанию) и **SYSCLK** (при желании)
- Используя HSI16, скорость UART не зависит от системной частоты, работает до 1МБод/с
- Используя SYSCLK и кварц можно добиться **большей** скорости, работает до 2МБод/с 

## Как использовать?
1. Добавь **stm32gx_uart.hpp** в свой проект 
2. Настрой **тактирование** системы (при помощи CubeMX или CMSIS)
3. Настрой **GPIO** выбранных UART в режим альт. функции **UARTx**
4. Инициализировать UART в CubeMX **не нужно!**
5. Создай объект(ы) UART (пример для **USART1**):
	```
    STM32G_UART <UART_FULL> uart(USART1);	
    ``` 
6. Создай обработчик(и) прерывания выбранного UART (пример для **USART1**):
	```
    extern "C" void USART1_IRQHandler () {
		uart.IRQ();
	}
    ``` 
7. Вызови метод **init()** (пример для 115200 Бод/с):
	```
    uart.init(115200); 
    ``` 
8. Выводи данные в порт при помощи **print()** или **printf()**:
	```
	uint32_t data = 1523142134;
	float value = 532.7613;
	
	uart.print(data); 
	uart.print(", ");
	uart.print(value);
	uart.println();
	
	uart.printf("long: %d, float: %f \r\n", data, value); 
    ```

9. Читай данные из порта при помощи **read()**:
	```
	while(uart.available()){
		uint8_t data = uart.read();
	}	
    ```
## Полный API:
### Создание объекта:
При создании объекта UART можно индивидуально указать:
- Режим работы (полудуплексный или полнодуплексный)
- Размер **приемного** буфера FIFO 
- Размер **передающего** буфера FIFO
	```
	STM32G_UART <UART_FULL> uart(LPUART1);			// Полнодуплексный, буферы: [256 байт RX / 256 байт TX]
	STM32G_UART <UART_FULL, 128, 512> uart(LPUART1);	// Полнодуплексный, буферы: [128 байт RX / 512 байт TX]
	STM32G_UART <UART_RX, 512, 1> uart(LPUART1);		// Только прием, буферы: [512 байт RX / 1 байт TX]
	STM32G_UART <UART_TX, 1, 512> uart(LPUART1);		// Только передача, буферы: [1 байт RX / 512 байт TX]
    ```
### Создание обработчика прерывания:
- Обработчик размещается в любом удобном файле
- Название корректируем под нужный UART
- В обработчике вызывается сервисный метод **IRQ()**
- Метод **IRQ()** можно дополнительно опрашивать в главном цикле
	```
	extern "C" void LPUART1_IRQHandler () { uart.IRQ(); }
    ```
### Инициализация:
При инициализации UART можно указать:
- Скорость порта **4800...2000000** Бод/с
- Частоту ядра, в этом случае UART будет тактироваться от **SYSCLK**
	```
	uart.init(115200);			// Скорость 115200, тактирование от HSE16
	uart.init(9600, 170E6);		// Скорость 9600, тактирование от системных 170МГц
    ```
### Системное:
- Сброс и отключение UART:	
	```
	uart.deInit();
    ```
- Изменение baudrate после инициализации:
	```
	uart.setBaudrate(9600);		// Вариант с тактированием от HSI16
	uart.setBaudrate(9600, 170E6);		// Вариант с тактированием от SYSCLK
    ```	
- Аппаратное включение и выключение приемника:
	```
	uart.rxEnable(1/0);	
    ```	
- Аппаратное включение и выключение передатчика:
	```
	uart.txEnable(1/0);	
    ```	
- Аппаратное включение и выключение UART:
	```
	uart.enable(1/0);	
    ```	
- Поменять контакты TX/RX местами (бывает удобно):
	```
	uart.swap(1/0);
    ```
- Полный аппаратный сброс UART:
	```
	uart.reset();
    ```
### Печать в порт:
