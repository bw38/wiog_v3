/*
 * shotlist.c
 *
 *  Created on: 06.12.2021
 *      Author: joerg
 */
/* Abschussliste - allgem Datenstruktur
 * -------------------------------------
 * init - Initialisierung der Datenstruktur, list von beliebigem ordenary type
 * free - freigeben der Datenstructur
 * add  - hinzufügen eines Wertes(keine doppelte Aufnahme) -> true / Überlauf der Liste, Wert nicht aufgenommen -> false
 * get  - Wert gefunden und gelöscht -> true / Wert nicht vorhanden -> false
 *
 * Werte in Datenstruktur shotlist_t dürfen nicht von außen geändert werden
 */

#include "shotlist.h"

void shotlist_init(shotlist_t *psl, int len) {
	psl->ix = 0;
	psl->top = len;
	psl->list = malloc(len * sizeof(psl->list[0]));
}

void shotlist_free(shotlist_t *psl) {
	free(psl->list);
}

bool shotlist_add(shotlist_t *psl, int32_t val) {
	//keine Änderung falls bereits Wert bereits vorhanden
	for (int i = 0; i < psl->ix; i++) {
		if (val == psl->list[i])
			return true;
	}
	//Wert oben eintragen, wenn Platz vorhanden
	if (psl->ix <= psl->top)	{
		psl->list[psl->ix] = val;
		psl->ix++;
		return true;
	}
	else return false;
}

bool shotlist_get(shotlist_t *psl, int32_t val){
	bool res = false;
	for (int i = 0; i < psl->ix; i++) {
		if (val == psl->list[i]) {
			psl->list[i] = psl->list[psl->ix];
			psl->ix--;
			res = true;
			break;
		}
	}
	return res;
}


