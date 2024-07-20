/*using System.Collections;
using UnityEngine;
using UnityEngine.AI;

public class Enemy : MonoBehaviour
{
    // Declaración de una variable privada para el agente NavMesh
    private NavMeshAgent agent;
    // Declaración de una variable privada para la posición del jugador
    private Transform player;

    // Variables para el cono de visión
    public float visionAngle = 45f;
    public float visionRange = 10f;

    // Variable para la posición inicial
    private Vector3 initialPosition;

    // Variable para controlar si el jugador está siendo perseguido
    private bool isChasing = false;

    // Start se llama antes de la primera actualización del frame
    void Start()
    {
        // Obtiene y guarda una referencia al componente NavMeshAgent adjunto a este GameObject
        agent = GetComponent<NavMeshAgent>();
        // Encuentra al objeto que tiene el script ClickToMove y obtiene su transform (posición y rotación)
        player = FindObjectOfType<ClickToMove>().transform;
        // Guarda la posición inicial del enemigo
        initialPosition = transform.position;
    }

    // Update se llama una vez por frame
    void Update()
    {
        if (!isChasing)
        {
            if (IsPlayerInVisionCone())
            {
                StartCoroutine(ChasePlayer());
            }
        }
    }

    // Método para verificar si el jugador está en el cono de visión
    bool IsPlayerInVisionCone()
    {
        Vector3 directionToPlayer = (player.position - transform.position).normalized;
        float angleToPlayer = Vector3.Angle(transform.forward, directionToPlayer);

        // Depuración para verificar el ángulo y la distancia
        Debug.Log("Angle to player: " + angleToPlayer);
        Debug.Log("Distance to player: " + Vector3.Distance(transform.position, player.position));

        if (angleToPlayer < visionAngle / 2 && Vector3.Distance(transform.position, player.position) <= visionRange)
        {
            return true;
        }
        return false;
    }

    // Corrutina para perseguir al jugador por 3 segundos
    IEnumerator ChasePlayer()
    {
        isChasing = true;
        float chaseDuration = 3f;
        float chaseTimer = 0f;

        while (chaseTimer < chaseDuration)
        {
            agent.SetDestination(player.position);
            chaseTimer += Time.deltaTime;
            yield return null;
        }

        // Regresar a la posición inicial
        agent.SetDestination(initialPosition);
        isChasing = false;
    }

    // Método para dibujar el cono de visión con Gizmos
    void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Vector3 forward = transform.forward * visionRange;
        Quaternion leftRayRotation = Quaternion.AngleAxis(-visionAngle / 2, Vector3.up);
        Quaternion rightRayRotation = Quaternion.AngleAxis(visionAngle / 2, Vector3.up);
        Vector3 leftRayDirection = leftRayRotation * forward;
        Vector3 rightRayDirection = rightRayRotation * forward;

        Gizmos.DrawRay(transform.position, forward);
        Gizmos.DrawRay(transform.position, leftRayDirection);
        Gizmos.DrawRay(transform.position, rightRayDirection);
        Gizmos.DrawWireSphere(transform.position, visionRange);
    }
}
*/

/*Fuentes de donde se consiguio ayuda para este script
 
    https://www.youtube.com/watch?v=HOAPvQONpsU
 
 */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class Enemy : MonoBehaviour
{
    // Declaración de una variable privada para el agente NavMesh
    private NavMeshAgent agent;
    // Declaración de una variable privada para la posición del jugador
    private Transform player;

    // Start se llama antes de la primera actualización del frame
    void Start()
    {
        // Obtiene y guarda una referencia al componente NavMeshAgent adjunto a este GameObject
        agent = GetComponent<NavMeshAgent>();
        // Encuentra al objeto que tiene el script ClickToMove y obtiene su transform (posición y rotación)
        player = FindAnyObjectByType<ClickToMove>().transform;
    }

    // Update se llama una vez por frame
    void Update()
    {
        // Establece el destino del agente NavMesh a la posición del jugador
        agent.SetDestination(player.position);
    }
}