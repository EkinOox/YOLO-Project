[//]: <> (https://stackedit.io/app#)

# Versioning GIT

## Bonnes pratiques en programmation.

### Générales

-   Lisibilité du code
    > Écrivez votre code de manière à ce qu'il soit lisible par d'autres.
-   Commentaires pertinents
    > Commentez le pourquoi, pas le comment. 1/3 de commentaire 2/3 de code
-   Nommer judicieusement
    > Utilisez des noms de variables, de fonctions et de classes qui décrivent leur utilité ou leur fonction. il faut pouvoir comprendre une fonction ou une variable uniquement grâce à son nom !
-   Diviser le code en fonctions et classes
    > Évitez les fonctions monolithiques. Divisez votre code en fonctions ou méthodes plus petites, chacune effectuant une tâche spécifique. Utilisez des classes pour regrouper des fonctionnalités liées.
-   Soyez conscient des implications en matière de sécurité
    > **Écrivez un code sécurisé, protégez-vous des vulnérabilités courantes.**

### Nommage des éléments :

-   Nommer judicieusement

    > Utilisez des noms de variables, de fonctions et de classes qui décrivent leur utilité ou leur fonction. il faut pouvoir comprendre une fonction ou une variable uniquement grâçe à son nom !

-   Syntaxe et indentation

    > propriétés (clés dans les objet js ou JSON) **snake_case** exemple : ma_propriete

    > variables (let) **camelCase** premier lettre en minuscule et tous les autres mots ont leur première lettre en maj parExempleCommeCa

    > constantes (const) **camelCase / CAPS** exeption pour les VRAIES CONSTANTES par exemple un délai d'attente en secondes qu'on notera tout en majuscule : const TIMER = 2

    > Nom des composants (nom du fichier .jsx) **PascalCase** par exemple le composant boutton envoyer : SendButton.jsx. PAS: le nom de la fonction interne au composant doit aussi etre en PC

-   Js:
    > variables & Fonction (let & function) **camelCase** premier lettre en minuscule et tous les autres mots ont leur premiere lettre en maj parExempleCommeCa
    
    > constantes (const) **camelCase / CAPS** exeption pour les VRAIES CONSTANTES par exemple un délai d'attente en secondes qu'on notera tout en majuscule : const TIMER = 2
    
    > Nom des composants (nom du fichier .jsx) **PascalCase** par exemple le composant bouton envoyer : SendButton.jsx. PAS: le nom de la fonction interne au composant doit aussi être en PC

    > class (class) **camelCase** première lettre en majuscule et tout les autres mots on leur premiere lettre en majuscule
```js
class User {}
const USER = "xxx";
function userConnected(userId) {}
const USER_ID = 2;
let height = 1.8;
```

-   HTMl:
    > classes, id (pour le HTML) **kebab-case** exemple : ma-super-div sans majuscule

```html
<div id="super-div" class="super-class"></div>;
```

-   CSS: 
    > classes, id (pour le CSS) **kebab-case** la même que le html

```css
.super-class {
    display : none;
}
```

### Structure du code :

-   Lisibilité du code
    > Écrivez votre code de manière à ce qu'il soit lisible par d'autres.
-   Commentaires pertinents
    > Commentez le pourquoi, pas le comment.
-   Diviser le code en fonctions et classes
    > Évitez les fonctions monolithiques. Divisez votre code en fonctions ou méthodes plus petites, chacune effectuant une tâche spécifique.
-   Éviter la duplication de code
    > Ne pas répéter le même code. Si vous avez besoin d'une logique similaire à plusieurs endroits, créez une fonction réutilisable.
    > Utilisez des classes pour regrouper des fonctionnalités liées.

### Gestion des erreurs et des exceptions :

-   Gérer les erreurs
    > Utilisez la gestion des exceptions pour gérer les erreurs plutôt que de renvoyer des codes d'erreur. Cela rend le code plus robuste et plus facile à maintenir.

### Performance et optimisation

-   Utiliser le même outil de mise en forme
    > Prettier

### Sécurité

-   Soyez conscient des implications en matière de sécurité
    > Écrivez un code sécurisé, protégez-vous des vulnérabilités courantes.
-   Nommage des variables
    > Utilisez des noms de variables qui décrivent leur utilité ou leur fonction. il faut pouvoir comprendre une variable uniquement grâce à son nom !
-   Syntaxe et indentation

    > variables (let) **camelCase** premier lettre en minuscule et tous les autres mots ont leur premiere lettre en maj parExempleCommeCa
    > Nom des composants (nom du fichier .jsx) **PascalCase** par exemple le composant bouton envoyer : SendButton.jsx. PAS: le nom de la fonction interne au composant doit aussi etre en PC
    > constantes (const) **camelCase** pour les conteneur. Exception pour les variables constante. par exemple un délai d'attente en secondes qu'on notera tout en majuscule : const TIMER = 2

-   Gérer les erreurs
    > Utilisez la gestion des exceptions pour gérer les erreurs plutôt que de renvoyer des codes d'erreur. Cela rend le code plus robuste et plus facile à maintenir.
-   Ne pas hardcoder les configurations
    > Utilisez des fichiers de configuration ou des variables d'environnement


### HTML

> classes, id (pour le HTML) **kebab-case** exemple : ma-super-div sans majuscule

### JS / JSON

> propriétés (clés dans les objet js ou JSON) **snake_case** exemple : ma_propriete

### Python
```python
class MyClass:
    def __init__(self, name):
        self.name = name

    def say_hello(self):
        print(f"Hello, {self.name}!")

my_variable = 1
MYCONSTANT = 2
```

### C#
    
```csharp
public class MyClass
{
    public MyClass(string name)
    {
        Name = name;
    }

    public string Name { get; set; }

    public void SayHello()
    {
        Console.WriteLine($"Hello, {Name}!");
    }
}
```

### PHP
```php
class MyClass
{
    public function __construct($name)
    {
        $this->name = $name;
    }

    public function sayHello()
    {
        echo "Hello, {$this->name}!";
    }
}
```
