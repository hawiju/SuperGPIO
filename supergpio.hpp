#include "stm32f4xx.h"
//#include "stm32f7xx.h"
namespace gpio {

enum class mode :int {
	analog = 0b11, input = 0b00, output = 0b01, output_af = 0b10
};
enum class type:int {
	PP = 0, OD = 1
};
enum class speed:int {
	low = 0, medium = 0b01, high = 0b10, very_high = 0b11
};
enum class pupd:int {
	nopupd = 0, pu = 0b01, pd = 0b10
};

enum class af:int {
	af0 = 0, af1, af2, af3, af4, af5, af6, af7, af8, af9, af10, af11, af12, af13, af14, af15
};
enum class pins:int {
	pin0 = 0, pin1, pin2, pin3, pin4, pin5, pin6, pin7, pin8, pin9, pin10, pin11, pin12, pin13, pin14, pin15
};

struct Pin{
	uint8_t pin
;
	//3 bytes go here
	GPIO_TypeDef* port;
};

#if __cpp_fold_expressions >= 201411 //true for GCC>=6
template<uint8_t ... pins>
inline void init(GPIO_TypeDef* port, mode m = mode::input, type t = type::PP, speed s = speed::low, pupd p = pupd::nopupd, af a = af::af0);

template<uint32_t ... pins>
inline void set(GPIO_TypeDef* port);

template<uint32_t ... pins>
inline void reset(GPIO_TypeDef* port);

template<uint32_t ... pins>
inline void toggle(GPIO_TypeDef* port);

#endif

inline void init(Pin pin, mode m = mode::input, type t = type::PP, speed s = speed::low, pupd p = pupd::nopupd, af a = af::af0);
inline void set(Pin p);
inline void reset(Pin p);
inline void toggle(Pin p);


}


#if __cpp_fold_expressions >= 201411
template<uint8_t ... pins>
inline void gpio::init(GPIO_TypeDef * port, mode m, type t, speed s, pupd p, af a) {

	if (port == GPIOA)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	if (port == GPIOB)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	if (port == GPIOC)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	if (port == GPIOD)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	if (port == GPIOE)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	if (port == GPIOF)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
	if (port == GPIOG)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
	if (port == GPIOH)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
	if (port == GPIOI)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
//	if (port == GPIOJ)                    //for big F7
//		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOJEN;
//	if (port == GPIOK)
//		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOKEN;

	uint32_t temp1 = 0, temp2 = 0;
	((temp1|=(0b11<<(2*pins))), ... );
	port->MODER &= ~(temp1);
	temp1 = 0;
	((temp1|=int(m)<<(2*pins)), ... );
	port->MODER |= temp1;
	temp1 = 0;
	((temp1|=(0b1<<(pins))), ... );
	port->OTYPER &= ~(temp1);
	temp1 = 0;
	((temp1|=(int(t)<<(pins))), ... );
	port->OTYPER |= temp1;
	temp1 = 0;
	((temp1|=(0b11<<(2*pins))), ... );
	port->OSPEEDR &= ~(temp1);
	temp1 = 0;
	((temp1|=(int(s)<<(2*pins))), ... );
	port->OSPEEDR |= temp1;
	temp1 = 0;
	((temp1|=(0b11<<(2*pins))), ... );
	port->PUPDR &= ~(temp1);
	temp1 = 0;
	((temp1|=(int(p)<<(2*pins))), ... );
	port->PUPDR |= temp1;
	temp1 = 0;
	(((pins<8?temp1:temp2)|=(0b11<<(4*(pins-(pins<8?0:8))))), ... );
	port->AFR[0] &= ~temp1;
	port->AFR[1] &= ~temp2;
	temp1 = 0;
	temp2 = 0;
	(((pins<8?temp1:temp2)|=(int(a)<<(4*(pins-(pins<8?0:8))))), ... );
	port->AFR[0] |= temp1;
	port->AFR[1] |= temp2;

}

template<uint32_t ... pins>
inline void gpio::set(GPIO_TypeDef* port) {
	uint32_t t = 0;
	((t |= 1<<pins), ...);
	port->BSRR = t;
}

template<uint32_t ... pins>
inline void gpio::toggle(GPIO_TypeDef* port) {
	uint32_t t = 0;
	((t |= 1<<pins), ...);
	port->ODR ^= t;
}

template<uint32_t ... pins>
inline void gpio::reset(GPIO_TypeDef* port) {
	uint32_t t = 0;
	((t |= 1<<(pins+16)), ...);
	port->BSRR = t;
}

#endif

inline void gpio::init(Pin pin, mode m , type t, speed s, pupd p , af a){

	if (pin.port == GPIOA)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	if (pin.port == GPIOB)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	if (pin.port == GPIOC)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	if (pin.port == GPIOD)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	if (pin.port == GPIOE)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	if (pin.port == GPIOF)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
	if (pin.port == GPIOG)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
	if (pin.port == GPIOH)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
	if (pin.port == GPIOI)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
//	if (pin.port == GPIOJ)                    //for big F7
//		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOJEN;
//	if (pin.port == GPIOK)
//		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOKEN;


	pin.port->MODER &= ~((0b11<<(2*pin.pin)));
	pin.port->MODER |= int(m)<<(2*pin.pin);

	pin.port->OTYPER &= ~((0b1<<(pin.pin)));
	pin.port->OTYPER |= (int(t)<<(pin.pin));

	pin.port->OSPEEDR &= ~((0b11<<(2*pin.pin)));
	pin.port->OSPEEDR |= (int(s)<<(2*pin.pin));
	
	pin.port->PUPDR &= ~((0b11<<(2*pin.pin)));
	pin.port->PUPDR |= (int(p)<<(2*pin.pin));
	
	pin.port->AFR[pin.pin<8?0:1]&= ~(0b11<<(4*(pin.pin-(pin.pin<8?0:8))));
	pin.port->AFR[pin.pin<8?0:1]|= (int(a)<<(4*(pin.pin-(pin.pin<8?0:8))));
}
inline void gpio::set(Pin p){
	p.port->BSRR=1<<p.pin;
}
inline void gpio::reset(Pin p){
	p.port->BSRR=1<<(p.pin+16);
}
inline void gpio::toggle(Pin p){
	p.port->ODR^=(1<<(p.pin));
}
