/*using System.Collections;
using UnityEngine;
using UnityEngine.AI;

public class Enemy : MonoBehaviour
{
    // Declaraci�n de una variable privada para el agente NavMesh
    private NavMeshAgent agent;
    // Declaraci�n de una variable privada para la posici�n del jugador
    private Transform player;

    // Variables para el cono de visi�n
    public float visionAngle = 45f;
    public float visionRange = 10f;

    // Variable para la posici�n inicial
    private Vector3 initialPosition;

    // Variable para controlar si el jugador est� siendo perseguido
    private bool isChasing = false;

    // Start se llama antes de la primera actualizaci�n del frame
    void Start()
    {
        // Obtiene y guarda una referencia al componente NavMeshAgent adjunto a este GameObject
        agent = GetComponent<NavMeshAgent>();
        // Encuentra al objeto que tiene el script ClickToMove y obtiene su transform (posici�n y rotaci�n)
        player = FindObjectOfType<ClickToMove>().transform;
        // Guarda la posici�n inicial del enemigo
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

    // M�todo para verificar si el jugador est� en el cono de visi�n
    bool IsPlayerInVisionCone()
    {
        Vector3 directionToPlayer = (player.position - transform.position).normalized;
        float angleToPlayer = Vector3.Angle(transform.forward, directionToPlayer);

        // Depuraci�n para verificar el �ngulo y la distancia
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

        // Regresar a la posici�n inicial
        agent.SetDestination(initialPosition);
        isChasing = false;
    }

    // M�todo para dibujar el cono de visi�n con Gizmos
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
    // Declaraci�n de una variable privada para el agente NavMesh
    private NavMeshAgent agent;
    // Declaraci�n de una variable privada para la posici�n del jugador
    private Transform player;

    // Start se llama antes de la primera actualizaci�n del frame
    void Start()
    {
        // Obtiene y guarda una referencia al componente NavMeshAgent adjunto a este GameObject
        agent = GetComponent<NavMeshAgent>();
        // Encuentra al objeto que tiene el script ClickToMove y obtiene su transform (posici�n y rotaci�n)
        player = FindAnyObjectByType<ClickToMove>().transform;
    }

    // Update se llama una vez por frame
    void Update()
    {
        // Establece el destino del agente NavMesh a la posici�n del jugador
        agent.SetDestination(player.position);
    }
}