# Rapport de Modélisation UPPAAL
## Système de Livraison par Drones

**📂 Repository GitHub** : [https://github.com/Nabil-Bkz/-Syst-me-de-Livraison-par-Drones](https://github.com/Nabil-Bkz/-Syst-me-de-Livraison-par-Drones)

---

## Table des Matières

1. [Introduction](#1-introduction)
2. [Déclarations Globales](#2-déclarations-globales)
3. [Template Drone](#3-template-drone)
4. [Template Controller](#4-template-controller)
5. [Template Meteo](#5-template-meteo)
6. [Système Global](#6-système-global)
7. [Propriétés et Vérification](#7-propriétés-et-vérification)
8. [Conclusion](#8-conclusion)

---

## 1. Introduction

### 1.1 Contexte

Ce projet modélise un système de livraison automatisé par drones dans un environnement portuaire. Les drones transportent des conteneurs depuis une plateforme de chargement vers une plateforme de déchargement.

### 1.2 Objectifs

- Modéliser le comportement des drones avec gestion de batterie
- Assurer l'exclusion mutuelle sur les plateformes (un seul drone à la fois)
- Gérer les situations d'urgence (batterie faible, tempête)
- Vérifier l'absence de deadlock et les propriétés de sûreté/vivacité

### 1.3 Composants du Système

| Processus | Rôle |
|-----------|------|
| `d1`, `d2` | Drones de livraison |
| `c` | Contrôleur central |
| `m` | Système météorologique |

---

## 2. Déclarations Globales

### 2.1 Constantes

```c
const int ND = 2;               // Nombre de drones
const int N_PLATFORMS = 2;      // Nombre de plateformes

// Constantes batterie
const int BAT_MAX = 100;        // Batterie maximale
const int BAT_COST = 5;         // Consommation par transition
const int BAT_CRITICAL = 10;    // Seuil critique : doit retourner charger
const int BAT_MIN_UNLOAD = 20;  // Seuil min pour aller vers déchargement
```

### 2.2 Variables

```c
int[0,1] missionType;           // Type de mission (0 ou 1)
int available[ND];              // Disponibilité des drones (1=disponible)
int platform_lock[N_PLATFORMS] = {-1, -1};  // Verrous plateformes (-1=libre)
bool isStorm = false;           // État météo global
```

### 2.3 Fonctions

```c
bool isPlatformFree(int p) {
    return platform_lock[p] == -1;
}

void lockPlatform(int p, int droneId) {
    platform_lock[p] = droneId;
}

void unlockPlatform(int p) {
    platform_lock[p] = -1;
}

int minBat(int a, int b) {
    return (a < b) ? a : b;
}
```

### 2.4 Canaux de Communication

```c
broadcast chan initCh;          // Initialisation des drones
broadcast chan missionCh[ND];   // Envoi de mission au drone i
broadcast chan stormStart;      // Signal début tempête
broadcast chan stormEnd;        // Signal fin tempête
```

---

## 3. Template Drone

### 3.1 Déclarations Locales

```c
int battery = BAT_MAX;  // Batterie initialisée à 100
clock t;                // Horloge pour le temps de charge
```

### 3.2 États

| État | Description |
|------|-------------|
| `Iddle` | État initial, attente d'initialisation |
| `Ready` | Prêt à recevoir une mission |
| `ToLoading` | En route vers plateforme de chargement |
| `WaitLoadPlatform` | Attente libération plateforme 0 |
| `Loading` | Chargement du conteneur |
| `ToUnloading` | En route vers plateforme de déchargement |
| `WaitUnloadPlatform` | Attente libération plateforme 1 |
| `Unloading` | Déchargement du conteneur |
| `Charging` | En recharge (invariant: `t <= 10`) |
| `SafeZone` | Zone de sécurité (tempête) |

### 3.3 Transitions Principales

| Transition | Guard | Sync | Assignment |
|------------|-------|------|------------|
| Iddle → Ready | - | `initCh?` | `available[id] = 1` |
| Ready → ToLoading | `battery > BAT_CRITICAL` | `missionCh[id]?` | `available[id] = 0, battery -= BAT_COST` |
| WaitLoadPlatform → Loading | `isPlatformFree(0) && battery > BAT_CRITICAL` | - | `lockPlatform(0, id)` |
| Loading → ToUnloading | `battery > BAT_MIN_UNLOAD` | - | `unlockPlatform(0)` |
| WaitUnloadPlatform → Unloading | `isPlatformFree(1) && battery > BAT_CRITICAL` | - | `lockPlatform(1, id)` |
| Unloading → Ready | `battery > BAT_CRITICAL` | - | `unlockPlatform(1), available[id] = 1` |
| Charging → Ready | `t >= 10 && battery >= 80 && !isStorm` | - | `battery = BAT_MAX, available[id] = 1` |
| * → SafeZone | - | `stormStart?` | (libère plateforme si occupée) |
| SafeZone → Ready | - | `stormEnd?` | `available[id] = 1` |

### 3.4 Gestion Batterie

- **Consommation** : -5 par transition
- **Seuil critique (≤10)** : Retour obligatoire à Charging
- **Seuil déchargement (≤20)** : Interdit d'aller vers ToUnloading
- **Recharge** : +20 par palier, durée minimum 10 ut

---

## 4. Template Controller

### 4.1 Déclarations Locales

```c
int[0,ND-1] cur;          // Indice du drone actuellement ciblé
bool initialized = false;  // True après initialisation des drones
```

### 4.2 États

| État | Description |
|------|-------------|
| `InitCtrl` | État initial |
| `ReadyCtrl` | Prêt à envoyer des missions |
| `StormMode` | Mode tempête, arrêt des envois |

### 4.3 Transitions

| Transition | Guard | Sync | Assignment |
|------------|-------|------|------------|
| InitCtrl → ReadyCtrl | - | `initCh!` | `initialized = true` |
| InitCtrl → StormMode | - | `stormStart?` | - |
| ReadyCtrl → ReadyCtrl | `available[i] == 1 && !isStorm` | `missionCh[i]!` | `cur = i, missionType = tt` |
| ReadyCtrl → StormMode | - | `stormStart?` | - |
| StormMode → ReadyCtrl | `initialized` | `stormEnd?` | - |
| StormMode → InitCtrl | `!initialized` | `stormEnd?` | - |

---

## 5. Template Meteo

### 5.1 Déclarations Locales

```c
clock tm;  // Horloge pour durée tempête
```

### 5.2 États

| État | Description |
|------|-------------|
| `Normal` | Conditions normales |
| `Storm` | Tempête en cours |

### 5.3 Transitions

| Transition | Guard | Sync | Assignment |
|------------|-------|------|------------|
| Normal → Storm | - | `stormStart!` | `isStorm = true, tm = 0` |
| Storm → Normal | `tm >= 10` | `stormEnd!` | `isStorm = false` |

---

## 6. Système Global

### 6.1 Instanciation

```c
d1 = drone(0);
d2 = drone(1);
c = controller();
m = meteo();

system d1, d2, c, m;
```

### 6.2 Fonctionnement Global

```
┌─────────────┐     initCh!      ┌─────────────┐
│  Controller │─────────────────▶│   Drones    │
│      c      │   missionCh[i]!  │  (d1, d2)   │
└─────────────┘                  └─────────────┘
       │                               │
       │ stormStart?                   │ stormStart?
       │ stormEnd?                     │ stormEnd?
       ▼                               ▼
┌─────────────┐                  ┌─────────────┐
│    Meteo    │──stormStart!────▶│  SafeZone   │
│      m      │──stormEnd!──────▶│             │
└─────────────┘                  └─────────────┘
```

### 6.3 Cycle de Mission

1. **Initialisation** : Controller envoie `initCh!` → Drones passent en Ready
2. **Envoi mission** : Controller envoie `missionCh[i]!` au drone disponible
3. **Exécution** : Drone suit le cycle ToLoading → Loading → ToUnloading → Unloading
4. **Fin** : Drone revient en Ready, `available[id] = 1`

### 6.4 Gestion Tempête

1. Meteo envoie `stormStart!`
2. Controller passe en StormMode
3. Drones vont en SafeZone (libèrent plateformes)
4. Après 10 ut, Meteo envoie `stormEnd!`
5. Système reprend normalement

---

## 7. Propriétés et Vérification

### 7.1 Propriétés de Sûreté (Safety)

| Propriété | Formule | Description |
|-----------|---------|-------------|
| Absence deadlock | `A[] not deadlock` | Pas de blocage global |
| Exclusion plateforme 0 | `A[] not (d1.Loading and d2.Loading)` | Un seul drone en chargement |
| Exclusion plateforme 1 | `A[] not (d1.Unloading and d2.Unloading)` | Un seul drone en déchargement |
| Batterie bornée | `A[] (d1.battery >= 0 and d1.battery <= BAT_MAX)` | Batterie dans [0,100] |
| Sécurité déchargement | `A[] (d1.ToUnloading imply d1.battery > BAT_CRITICAL)` | Batterie suffisante |
| Cohérence météo | `A[] (m.Storm imply (c.StormMode or c.InitCtrl))` | Controller réagit à tempête |

### 7.2 Propriétés de Vivacité (Liveness)

| Propriété | Formule | Description |
|-----------|---------|-------------|
| Charge possible | `E<> d1.Charging` | Drone peut se recharger |
| Tempête possible | `E<> m.Storm` | Tempête peut survenir |
| SafeZone accessible | `E<> d1.SafeZone` | Drone peut aller en sécurité |
| Livraison possible | `E<> d1.Unloading` | Drone peut livrer |

### 7.3 Tableau Récapitulatif

| # | Propriété | Résultat |
|---|-----------|----------|
| 1 | `A[] not deadlock` | ✅ |
| 2 | `A[] not (d1.Loading and d2.Loading)` | ✅ |
| 3 | `A[] not (d1.Unloading and d2.Unloading)` | ✅ |
| 4 | `A[] (d1.battery >= 0 and d1.battery <= BAT_MAX)` | ✅ |
| 5 | `A[] (d2.battery >= 0 and d2.battery <= BAT_MAX)` | ✅ |
| 6 | `A[] (d1.ToUnloading imply d1.battery > BAT_CRITICAL)` | ✅ |
| 7 | `A[] (d2.ToUnloading imply d2.battery > BAT_CRITICAL)` | ✅ |
| 8 | `A[] (m.Storm imply (c.StormMode or c.InitCtrl))` | ✅ |
| 9 | `E<> d1.Charging` | ✅ |
| 10 | `E<> d2.Charging` | ✅ |
| 11 | `E<> m.Storm` | ✅ |
| 12 | `E<> d1.SafeZone` | ✅ |
| 13 | `E<> d2.SafeZone` | ✅ |
| 14 | `E<> d1.Unloading` | ✅ |
| 15 | `E<> d2.Unloading` | ✅ |

---

## 8. Conclusion

### 8.1 Résumé

Nous avons modélisé un système de livraison par drones comprenant :
- **2 drones** avec gestion de batterie
- **1 contrôleur** pour l'envoi de missions
- **1 système météo** pour les tempêtes
- **2 plateformes** avec exclusion mutuelle

### 8.2 Résultats

Toutes les **15 propriétés** ont été vérifiées avec succès :
- **8 propriétés de sûreté** (A[])
- **7 propriétés de vivacité** (E<>)

### 8.3 Points Clés

1. **Pas de deadlock** : Le système progresse toujours
2. **Exclusion mutuelle** : Un seul drone par plateforme
3. **Gestion batterie** : Aucun drone ne reste bloqué
4. **Réactivité tempête** : Tous les composants réagissent correctement

---

*Rapport - Projet UPPAAL - Système de Livraison par Drones*
