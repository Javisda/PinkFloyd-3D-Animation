using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

public class Personaje : MonoBehaviour
{
    // Referencia al Script del Alumno para poder golpearle
    public SolidBehaviour[] studentSolids;
    public Transform[] studentTransforms;

    private Animator anim;
    private Rigidbody rb;

    public float velocidadMovimiento = 40.0f;
    public float velocidadRotacion = 200.0f;
    public float x, y;

    public bool canMove; // Variable para poder cambiar entre camaras

    public AudioManager audioManager;


    void Start()
    {
        anim = GetComponent<Animator>();
        rb = GetComponent<Rigidbody>();
        canMove = true;
    }


    void Update()
    {
        if (canMove) {
            x = Input.GetAxis("Horizontal");
            y = Input.GetAxis("Vertical");

            if (!anim.GetBool("DecirFrase"))
            {
                transform.Rotate(0, x * Time.deltaTime * velocidadRotacion, 0);
                transform.Translate(0, 0, y * Time.deltaTime * velocidadMovimiento);
            }

            anim.SetFloat("VelX", x);
            anim.SetFloat("VelY", y);

            Animaciones();
        }

        // Cambio entre control de personaje y cámara
        if(Input.GetKeyDown(KeyCode.H)){
            SwapMove();
        }
    }

    void Animaciones() {
        // Correr
        if (Input.GetKeyDown(KeyCode.LeftShift) && IsMoving())
        {
            FalseIdles();
            anim.SetBool("Correr", true);
            audioManager.Play("Correr");
            velocidadMovimiento = 60.0f;
        }
        if (Input.GetKeyUp(KeyCode.LeftShift))
        {
            anim.SetBool("Correr", false);
            audioManager.Stop("Correr");
            PickRandomIdle();
            velocidadMovimiento = 40.0f;
        }

        // Decir Frase
        if (Input.GetKeyDown(KeyCode.Y) && !IsMoving())
        {
            FalseIdles();
            anim.SetBool("DecirFrase", true);
            audioManager.Play("Frase");
        }
        if (Input.GetKeyUp(KeyCode.Y))
        {
            anim.SetBool("DecirFrase", false);
            audioManager.Stop("Frase");
            PickRandomIdle();
        }

        // Roll.
        if (Input.GetKeyDown(KeyCode.Space) && anim.GetBool("Correr")) {
            anim.SetTrigger("Roll");
        }

        // Attack
        if (Input.GetMouseButtonDown(0)) {
            PickRandomAttack();
            for (int i = 0; i < studentTransforms.Length; i++) { 
                if (Vector3.Distance(transform.position, studentTransforms[i].position) < 20.0f) {
                    Vector3 hitDirection = studentTransforms[i].position - transform.position;
                    hitDirection.Normalize();
                    studentSolids[i].ExternalForceDirection = hitDirection;
                    StartCoroutine(HitAfterTime(studentSolids[i]));
                }
            }
        }
    }

    private void FalseIdles() { 
        anim.SetBool("Idle1", false);
        anim.SetBool("Idle2", false);
        anim.SetBool("Idle3", false);
    }

    private void PickRandomIdle() {
        float election = Random.RandomRange(0.0f, 3.0f);
        if (election < 1.0f){
            anim.SetBool("Idle1", true);
        } else if (election >= 1.0f && election < 2.0f) {
            anim.SetBool("Idle2", true);
        } else if (election >= 2.0f) {
            anim.SetBool("Idle3", true);
        }
    }

    private void PickRandomAttack() {
        float election = Random.RandomRange(0.0f, 1.0f);
        if (election < 0.5)
        {
            anim.SetBool("Attack1", true);
            audioManager.Play("Swoosh4");
        }
        else if (election >= 0.5)
        {
            anim.SetBool("Attack2", true);
            audioManager.Play("Swoosh3");
        }
    }

    private bool IsMoving() {
        if (x != 0 || y != 0) {
            return true;
        }
        return false;
    }

    public void SwapMove() {
        canMove = !canMove;
    }


    IEnumerator HitAfterTime(SolidBehaviour s) {
        yield return new WaitForSeconds(0.5f);
        s.ApplyForce = true;
    }
}
