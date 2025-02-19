#ifndef _UTIL_WORD_H_
#define _UTIL_WORD_H_

/**
 * @file
 * Funciones utilitarias para manipular enteros de 32 bits en palabras de 16 bits.
 *
 * Estas funciones permiten extraer el word bajo y el word alto de un entero de 32 bits.
 *
 * @defgroup util_word Funciones utilitarias para manipular words
 * @code
 *   #include "util/word.h"
 * @endcode
 */

/**
 * Retorna el word bajo (16 bits) de un entero de 32 bits.
 *
 * @param ww Entero de 32 bits.
 * @return Los 16 bits menos significativos del entero.
 */
static inline uint16_t lowWord(uint32_t ww)
{
  return static_cast<uint16_t>(ww & 0xFFFF);
}

/**
 * Retorna el word alto (16 bits) de un entero de 32 bits.
 *
 * @param ww Entero de 32 bits.
 * @return Los 16 bits más significativos del entero.
 */
static inline uint16_t highWord(uint32_t ww)
{
  return static_cast<uint16_t>(ww >> 16);
}

#endif /* _UTIL_WORD_H_ */