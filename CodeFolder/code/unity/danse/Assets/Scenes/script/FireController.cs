using UnityEngine;

public class FireController : MonoBehaviour
{
    public UdpSocket udpSocket; // Assurez-vous que cette référence est correctement configurée dans l'éditeur Unity
    public GameObject magicFirePrefab; // Faites glisser votre prefab MagicFire ici dans l'éditeur Unity

    private ParticleSystem sparksParticles;
    private ParticleSystem fireParticles;
    private ParticleSystem smokeParticles;
    private ParticleSystem darkFireParticles;

    // Stockez les tailles initiales et les bornes de la variation aléatoire
    private float initialSparksSize;
    private float initialFireSize;
    private float initialSmokeSize;
    private float initialDarkFireSize;

    private float sparksRandomRangeMin;
    private float sparksRandomRangeMax;
    private float fireRandomRangeMin;
    private float fireRandomRangeMax;
    private float smokeRandomRangeMin;
    private float smokeRandomRangeMax;
    private float darkFireRandomRangeMin;
    private float darkFireRandomRangeMax;

    void Start()
    {
        // Assurez-vous que le prefab MagicFire a un ParticleSystem attaché pour les étincelles
        sparksParticles = magicFirePrefab.transform.Find("Sparks").GetComponent<ParticleSystem>();
        // initialSparksSize = sparksParticles.main.startSize.constant;
        sparksRandomRangeMin = sparksParticles.main.startSize.constantMin;
        sparksRandomRangeMax = sparksParticles.main.startSize.constantMax;

        // Assurez-vous que le prefab MagicFire a un ParticleSystem attaché pour le feu
        fireParticles = magicFirePrefab.transform.Find("Fire").GetComponent<ParticleSystem>();
        // initialFireSize = fireParticles.main.startSize.constant;
        fireRandomRangeMin = fireParticles.main.startSize.constantMin;
        fireRandomRangeMax = fireParticles.main.startSize.constantMax;

        // Assurez-vous que le prefab MagicFire a un ParticleSystem attaché pour la fumée
        smokeParticles = magicFirePrefab.transform.Find("Smoke").GetComponent<ParticleSystem>();
        // initialSmokeSize = smokeParticles.main.startSize.constant;
        smokeRandomRangeMin = smokeParticles.main.startSize.constantMin;
        smokeRandomRangeMax = smokeParticles.main.startSize.constantMax;

        // Assurez-vous que le prefab MagicFire a un ParticleSystem attaché pour le DarkFire
        darkFireParticles = magicFirePrefab.transform.Find("FireDark").GetComponent<ParticleSystem>();
        // initialDarkFireSize = darkFireParticles.main.startSize.constant;
        darkFireRandomRangeMin = darkFireParticles.main.startSize.constantMin;
        darkFireRandomRangeMax = darkFireParticles.main.startSize.constantMax;
    }

    void Update()
    {
        // Assurez-vous que le script UdpSocket est configuré

        // Obtenez la valeur actuelle du niveau sonore
        float soundLevel = udpSocket.soundLevel;

        // Vérifiez si le niveau sonore dépasse 70 décibels
        if (soundLevel > 70f)
        {
            // Calculez une nouvelle valeur pour startSize en fonction du niveau sonore
            float newSize = Remap(soundLevel, 70f, 100f, 1.0f, 5.0f);

            // Multipliez les tailles par deux
            newSize *= 5f;

            // Modifiez la taille des particules des étincelles
            var mainSparks = sparksParticles.main;
            // mainSparks.startSize = Random.Range(newSize, newSize * 5f);
            mainSparks.startSize = Random.Range(sparksRandomRangeMin * 5f, sparksRandomRangeMax * 5f);

            // Modifiez la taille des particules du feu
            var mainFire = fireParticles.main;
            // mainFire.startSize = Random.Range(newSize, newSize * 10f);
            mainFire.startSize = Random.Range(fireRandomRangeMin * 10f, fireRandomRangeMax * 10f);

            // Modifiez la taille des particules de la fumée
            var mainSmoke = smokeParticles.main;
            // mainSmoke.startSize = Random.Range(newSize, newSize * 5f);
            mainSmoke.startSize = Random.Range(smokeRandomRangeMin * 5f, smokeRandomRangeMax * 5f);

            // Modifiez la taille des particules du DarkFire
            var mainDarkFire = darkFireParticles.main;
            // mainDarkFire.startSize = Random.Range(newSize, newSize * 5f);
            mainDarkFire.startSize = Random.Range(darkFireRandomRangeMin * 5f, darkFireRandomRangeMax * 5f);

            // Modifiez la taille du prefab principal MagicFire
            var mainMagicFire = magicFirePrefab.GetComponent<ParticleSystem>().main;
            mainMagicFire.startSize = Random.Range(newSize, newSize * 5f);
        }
        else if (soundLevel < 80f)
        {
            // Réinitialisez les bornes de la variation aléatoire aux valeurs par défaut
            var mainSparks = sparksParticles.main;
            mainSparks.startSize = Random.Range(sparksRandomRangeMin, sparksRandomRangeMax);

            var mainFire = fireParticles.main;
            mainFire.startSize = Random.Range(fireRandomRangeMin, fireRandomRangeMax);

            var mainSmoke = smokeParticles.main;
            mainSmoke.startSize = Random.Range(smokeRandomRangeMin, smokeRandomRangeMax);

            var mainDarkFire = darkFireParticles.main;
            mainDarkFire.startSize = Random.Range(darkFireRandomRangeMin, darkFireRandomRangeMax);

            // Réinitialisez la taille du prefab principal MagicFire
            var mainMagicFire = magicFirePrefab.GetComponent<ParticleSystem>().main;
            // mainMagicFire.startSize = Random.Range(initialSparksSize, initialSparksSize * 5f);
            mainMagicFire.startSize = 1.7f;
        }
    }

    // Fonction pour remapper une valeur d'une plage à une autre
    float Remap(float value, float from1, float to1, float from2, float to2)
    {
        return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
    }
}
